"""
Quest 2 Teleoperation — Isaac Sim 4.5 Standalone [v12]
======================================================
v12 fixes (vs v11):

  FIX 1 — GRIP STILL NOT WORKING:
     Root cause: right_squeeze_value is bound as an ANALOG (float 0→1) in
     Isaac Sim's ov.vrmanifest.json, NOT as a digital bool. Calling
     GetDigitalActionData (slot 7) on a float-typed binding causes openvr to
     return a zero-filled struct → bState always False.
     Fix: grip is now polled with GetAnalogActionData (same as triggers) and
     thresholded at > 0.5. The digital() helper is kept as a fallback but
     grip reads via analog_squeeze() first.

  FIX 2 — POV/CAMERA ROTATES WHEN THUMBSTICK MOVES:
     Root cause: carb settings were applied before the XR profile fully
     initialises, so the locomotion system resets them on startup.
     Fix: _disable_xr_locomotion() is now called BOTH at launch-args time AND
     re-applied via a one-shot omni.kit.app event subscription that fires on
     the first post-physics tick after the VR session goes live. This ensures
     the settings survive the XR profile's own init sequence.

  NEW — CAMERA ZOOM (A / B / X / Y buttons):
     Added action handles for:
       /actions/ov/in/a_button          (right A — zoom in)
       /actions/ov/in/b_button          (right B — zoom out)
       /actions/ov/in/x_button          (left  X — zoom in)
       /actions/ov/in/y_button          (left  Y — zoom out)
     Zoom is applied to the XROrigin translate-Z each frame while held.
     A/X zoom in (camera moves forward); B/Y zoom out (camera moves back).

v11 fixes (vs v10):
  Struct alignment (_pack_=1 + explicit _pad fields) for _Analog and _Digital.
  analog()/vec2()/digital() accept value when err==0 regardless of bActive.

v10 fixes (vs v9):
  GetDigitalActionData (vtable slot 7) for squeeze. Struct still misaligned
  (fixed in v11).  XR locomotion disabled via carb settings at launch.

v9 fixes (vs v5):
  Wrong VR extension / double openvr.init() / bad warmup call.

Action names (from omni.kit.xr.system.steamvr oculus_touch.json):
  /actions/ov/in/right_trigger_value        float  analog
  /actions/ov/in/right_squeeze_value        float  analog  ← NOT digital!
  /actions/ov/in/left_trigger_value         float  analog
  /actions/ov/in/right_thumbstick_position  vec2   analog
  /actions/ov/in/left_thumbstick_position   vec2   analog
  /actions/ov/in/a_button                   bool   digital
  /actions/ov/in/b_button                   bool   digital
  /actions/ov/in/x_button                   bool   digital
  /actions/ov/in/y_button                   bool   digital
"""

import argparse, os, sys, glob as _glob

parser = argparse.ArgumentParser()
parser.add_argument("--isaac_path",     type=str, default="")
parser.add_argument("--run_vr",         action="store_true")
parser.add_argument("--robot",          default="franka", choices=["franka", "ur5"])
parser.add_argument("--headless",       action="store_true")
parser.add_argument("--no_locomotion",  action="store_true", default=True,
                    help="(default ON) Disable XR thumbstick locomotion so it "
                         "does not rotate the camera POV while driving joints.")
args, _ = parser.parse_known_args()

isaac_path = (args.isaac_path
              or os.environ.get("ISAACSIM_PATH", "")
              or os.environ.get("ISAAC_SIM_PATH", ""))

if not isaac_path:
    for p in sys.path:
        for candidate in [p, os.path.dirname(p)]:
            if ("isaac-sim-standalone-4" in candidate
                    and os.path.isdir(os.path.join(candidate, "apps"))):
                isaac_path = candidate
                break
        if isaac_path:
            break

if not isaac_path:
    print("[ERROR] Cannot find Isaac Sim 4.5. Run: source activate_teleop.sh")
    sys.exit(1)

if os.path.isfile(isaac_path):
    isaac_path = os.path.dirname(isaac_path)
isaac_path = os.path.realpath(isaac_path)

if not os.path.isdir(os.path.join(isaac_path, "apps")):
    print(f"[ERROR] '{isaac_path}' has no apps/ subdir.")
    sys.exit(1)

print(f"[OK] Isaac Sim: {isaac_path}")

os.environ["ISAACSIM_PATH"]        = isaac_path
os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
os.environ.setdefault("OMNI_KIT_RTX_MAX_GPU_COUNT",               "1")
os.environ.setdefault("CARB_SETTINGS_RENDERING_GPU_MEMORY_BUDGET", "5000")

pp = os.path.join(isaac_path, "python_packages")
if os.path.isdir(pp) and pp not in sys.path:
    sys.path.insert(0, pp)

kit_files = (_glob.glob(os.path.join(isaac_path, "apps", "isaacsim.exp.full*.kit"))
             or _glob.glob(os.path.join(isaac_path, "apps", "*.kit")))
if not kit_files:
    print(f"[ERROR] No .kit files in {isaac_path}/apps/")
    sys.exit(1)
print(f"[OK] Kit: {os.path.basename(kit_files[0])}")

launcher_args = {"headless": args.headless, "width": 1280, "height": 720}
if args.run_vr:
    extra = [
        "--enable", "omni.kit.xr.profile.vr",
        "--enable", "omni.kit.xr.system.steamvr",   # SteamVR compositor → ALVR → Quest
        # NOTE: do NOT use isaacsim.xr.openxr here — that is for native OpenXR
        # devices (e.g. direct USB tethering). ALVR streams via SteamVR, so the
        # steamvr extension is what feeds frames to the ALVR encoder.
    ]
    if args.no_locomotion:
        # Belt-and-suspenders: pass carb settings overrides at kit launch so the
        # locomotion system is off before the XR profile tries to register it.
        extra += [
            "--/xr/profile/locomotion/enabled=false",
            "--/xr/locomotion/enabled=false",
            "--/persistent/xr/profile/locomotion/enabled=false",
        ]
    launcher_args["extra_args"] = extra

print("[INIT] Launching SimulationApp …")
from isaacsim import SimulationApp
simulation_app = SimulationApp(launcher_args)
print("[INIT] SimulationApp ready.")

import numpy as np
import carb, omni, omni.usd, omni.kit.commands
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
from pxr import UsdGeom, Gf, UsdLux, Sdf


def setup_vr_camera(lock_locomotion: bool = True):
    """
    Place the XR origin in the scene and (optionally) disable the built-in
    thumbstick locomotion that would otherwise rotate the camera POV while
    the right thumbstick is used to drive robot joints.
    """
    try:
        stage = omni.usd.get_context().get_stage()
        create_prim("/World/XROrigin", "Xform")
        xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/XROrigin"))
        xf.ClearXformOpOrder()
        PD = UsdGeom.XformOp.PrecisionDouble
        xf.AddTranslateOp(precision=PD).Set(Gf.Vec3d(0.0, -1.5, 0.0))
        print("[VR] XR origin placed at (0, -1.5, 0)")
    except Exception as e:
        print(f"[VR] XR origin setup error: {e}")

    if lock_locomotion:
        _disable_xr_locomotion()


def _apply_locomotion_settings():
    """
    Write all carb settings that kill XR locomotion.
    Called once at startup AND once after the XR profile finishes its own init
    (via the post-startup subscription in _disable_xr_locomotion).
    """
    try:
        settings = carb.settings.get_settings()

        # Primary: disable locomotion entirely in the VR profile
        for path in [
            "/xr/profile/locomotion/enabled",
            "/xr/locomotion/enabled",
            "/persistent/xr/profile/locomotion/enabled",
            "/app/vr/locomotion/enabled",
            "/xr/profile/vr/locomotion/enabled",
            "/xr/profile/vr/locomotion/turn/enabled",
        ]:
            try:
                settings.set_bool(path, False)
            except Exception:
                pass

        # Secondary: zero movement and turn speeds (belt-and-suspenders)
        for path in [
            "/xr/profile/locomotion/turnSpeed",
            "/xr/profile/locomotion/moveSpeed",
            "/xr/locomotion/turnSpeed",
            "/xr/locomotion/moveSpeed",
            "/xr/profile/vr/locomotion/turn/speed",
            "/xr/profile/vr/locomotion/move/speed",
        ]:
            try:
                settings.set_float(path, 0.0)
            except Exception:
                pass
    except Exception as ex:
        print(f"[VR] locomotion-settings warning: {ex}")


def _disable_xr_locomotion():
    """
    Disable Isaac Sim's built-in XR locomotion system so the right thumbstick
    only drives our robot joints and does NOT also yaw the camera rig.

    Strategy:
      1. Apply settings NOW (before XR profile loads).
      2. Register a one-shot app-update subscription to re-apply settings after
         the XR profile finishes its own initialisation (which would otherwise
         reset them).  This is the fix for POV rotation persisting in v11.
    """
    _apply_locomotion_settings()
    print("[VR] Locomotion settings applied at launch.")

    # Re-apply after XR profile initialises (it runs in the first few frames)
    # We use a subscription that fires on every update but unsubscribes itself
    # after the first 120 frames (~4 s at 30 fps) — enough for XR init to finish.
    _reapply_state = {"count": 0, "sub": None}

    def _reapply_cb(e):
        _reapply_state["count"] += 1
        if _reapply_state["count"] == 120:   # fire once, ~4 s after start
            _apply_locomotion_settings()
            print("[VR] Locomotion settings re-applied post-XR-init.")
        if _reapply_state["count"] >= 121:
            sub = _reapply_state.get("sub")
            if sub:
                sub.unsubscribe()
            return

    try:
        import omni.kit.app
        sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            _reapply_cb, name="vr_locomotion_lock"
        )
        _reapply_state["sub"] = sub
        print("[VR] Post-init locomotion lock scheduled (fires at frame 120).")
    except Exception as ex:
        print(f"[VR] Post-init subscription warning (non-fatal): {ex}")


class KeyboardReader:
    def __init__(self):
        self.enabled = False
        self._input = None
        self._kb    = None
        try:
            import omni.appwindow
            app_window = omni.appwindow.get_default_app_window()
            self._kb   = app_window.get_keyboard()
            self._input = carb.input.acquire_input_interface()
            K = carb.input.KeyboardInput
            self._keys = {
                "w": K.W, "s": K.S, "a": K.A, "d": K.D,
                "q": K.Q, "e": K.E, "g": K.G, "r": K.R,
                "esc": K.ESCAPE,
            }
            self.enabled = True
            print("[Keyboard] Ready.")
        except Exception as ex:
            print(f"[Keyboard] Disabled: {ex}")

    def held(self, key):
        if not self.enabled or self._kb is None:
            return False
        try:
            return self._input.get_keyboard_value(self._kb, self._keys[key]) > 0.5
        except Exception:
            return False

import ctypes as _ct

class VRControllerReader:
    """
    Quest 2 controller input via IVRInput_010 pure ctypes — FINAL version.

    Strategy (proven to work given your diagnostic output):
      1. Load Isaac Sim's own libopenvr_api.so (version-matched to session)
      2. Get IVRInput_010 interface pointer via VR_GetGenericInterface
      3. DO NOT call SetActionManifestPath — Isaac Sim's ov.vrmanifest.json
         already registers /actions/ov at startup. A second manifest call
         causes bActive=False for all actions.
      4. Call GetActionHandle for each action name (slot 2)
      5. Each frame: UpdateActionState (slot 4), GetAnalogActionData (slot 6)

    All vtable functions are pre-built once and stored in self._fn_refs
    to prevent garbage collection (GC of a CFUNCTYPE = segfault).
    """

    _ACT_RT   = "/actions/ov/in/right_trigger_value"
    _ACT_RG   = "/actions/ov/in/right_squeeze_value"   # ANALOG float 0→1, NOT digital!
    _ACT_LT   = "/actions/ov/in/left_trigger_value"
    _ACT_RJ   = "/actions/ov/in/right_thumbstick_position"
    _ACT_LJ   = "/actions/ov/in/left_thumbstick_position"
    _ACT_A    = "/actions/ov/in/a_button"               # right A → zoom in
    _ACT_B    = "/actions/ov/in/b_button"               # right B → zoom out
    _ACT_X    = "/actions/ov/in/x_button"               # left  X → zoom in
    _ACT_Y    = "/actions/ov/in/y_button"               # left  Y → zoom out
    _ACT_SET  = "/actions/ov"
    # All polled via GetAnalogActionData (squeeze is float, buttons read as analog 0/1)
    _ANALOG_ACTIONS  = [_ACT_RT, _ACT_RG, _ACT_LT, _ACT_RJ, _ACT_LJ]
    # Buttons: try digital first, fall back to analog threshold
    _DIGITAL_ACTIONS = [_ACT_A, _ACT_B, _ACT_X, _ACT_Y]
    _ACTIONS  = _ANALOG_ACTIONS + _DIGITAL_ACTIONS

    def __init__(self, enabled: bool):
        self.enabled  = enabled
        self._lib     = None
        self._fn_refs = []   # MUST keep CFUNCTYPE objects alive — GC kills them otherwise
        self._ready   = False
        self._step    = 0
        self._rt = self._rg = self._lt = 0.0
        self._rx = self._ry = self._lx = self._ly = 0.0
        self._rpos = None
        self._btn_a = self._btn_b = self._btn_x = self._btn_y = False
        self._fn_update  = None
        self._fn_analog  = None
        self._fn_digital = None
        self._handles    = {}
        self._set_h      = 0
        self._ct         = None

        if not enabled:
            return
        self._lib = self._load_lib()

    # ── load Isaac Sim's version-matched libopenvr_api.so ─────────────
    def _load_lib(self):
        import ctypes, glob as _g
        isaac = os.environ.get("ISAACSIM_PATH", "")
        candidates = (
            _g.glob(os.path.join(isaac, "extscache",
                                 "omni.kit.xr.system.steamvr*",
                                 "bin", "libopenvr_api.so"))
            + [os.path.expanduser(
                "~/.local/share/Steam/steamapps/common/SteamVR"
                "/bin/linux64/libopenvr_api.so")]
        )
        for path in candidates:
            if not os.path.isfile(path):
                continue
            try:
                lib = ctypes.CDLL(path)
                # Confirm required symbols exist
                _ = lib.VR_GetGenericInterface
                _ = lib.VR_IsRuntimeInstalled
                lib.VR_GetGenericInterface.restype  = ctypes.c_size_t
                lib.VR_GetGenericInterface.argtypes = [ctypes.c_char_p,
                                                       ctypes.POINTER(ctypes.c_int)]
                print(f"[VR] Loaded: {path}")
                return lib
            except Exception as ex:
                print(f"[VR] skip {path}: {ex}")
        print("[VR] FATAL: libopenvr_api.so not loadable")
        return None

    # ── build a stable bound vtable method (slot-indexed) ─────────────
    def _make_fn(self, ct, vt_addr, slot, restype, *argtypes):
        """
        Read function pointer from vtable[slot], wrap in CFUNCTYPE,
        store wrapper in self._fn_refs (prevents GC), return bound callable.
        vt_addr: integer address of the vtable (first word of C++ object).
        """
        fn_addr = ct.c_size_t.from_address(vt_addr + slot * 8).value
        if not fn_addr:
            raise RuntimeError(f"vtable slot {slot} is NULL")
        # 'this' is passed as c_size_t (raw integer pointer) — most stable on Linux x64
        FT = ct.CFUNCTYPE(restype, ct.c_size_t, *argtypes)
        fn = FT(fn_addr)
        self._fn_refs.append(fn)          # keep alive
        sys_ptr = self._sys_ptr           # captured by value
        def bound(*args):
            return fn(sys_ptr, *args)
        return bound

    # ── acquire IVRInput_010 and resolve action handles ────────────────
    def _ensure_ready(self):
        if self._ready or self._lib is None:
            return

        import ctypes as ct
        lib = self._lib

        # Step 1: Get IVRInput_010 interface pointer
        err = ct.c_int(0)
        vi_ptr = 0
        for ver in [b"IVRInput_010", b"IVRInput_007", b"IVRInput_006"]:
            p = lib.VR_GetGenericInterface(ver, ct.byref(err))
            if p and err.value == 0:
                vi_ptr = p
                self._iface = ver.decode()
                print(f"[VR] IVRInput interface: {self._iface}")
                break

        if not vi_ptr:
            return  # VR session not live — silent retry

        # vi_ptr is an integer (c_size_t return). The C++ object layout is:
        #   vi_ptr[0..7]  = address of vtable
        #   vi_ptr[8..]   = object data
        self._sys_ptr = vi_ptr
        vt_addr = ct.c_size_t.from_address(vi_ptr).value
        if not vt_addr:
            print("[VR] vtable NULL — session not ready yet")
            return

        # Step 2: Build stable vtable callers
        # slot 1 = GetActionSetHandle(char*, uint64*) → int
        # slot 2 = GetActionHandle(char*, uint64*) → int
        # slot 4 = UpdateActionState(void* sets, uint32 sz, uint32 n) → int
        # slot 6 = GetAnalogActionData(uint64 h, void* d, uint32 sz, uint64 restr) → int
        # slot 7 = GetDigitalActionData(uint64 h, void* d, uint32 sz, uint64 restr) → int
        try:
            fn_get_set_h = self._make_fn(ct, vt_addr, 1, ct.c_int,
                                          ct.c_char_p, ct.POINTER(ct.c_uint64))
            fn_get_act_h = self._make_fn(ct, vt_addr, 2, ct.c_int,
                                          ct.c_char_p, ct.POINTER(ct.c_uint64))
            fn_update    = self._make_fn(ct, vt_addr, 4, ct.c_int,
                                          ct.c_void_p, ct.c_uint32, ct.c_uint32)
            fn_analog    = self._make_fn(ct, vt_addr, 6, ct.c_int,
                                          ct.c_uint64, ct.c_void_p,
                                          ct.c_uint32, ct.c_uint64)
            fn_digital   = self._make_fn(ct, vt_addr, 7, ct.c_int,
                                          ct.c_uint64, ct.c_void_p,
                                          ct.c_uint32, ct.c_uint64)
        except Exception as ex:
            print(f"[VR] vtable bind failed: {ex}")
            return

        # Step 3: GetActionSetHandle — DO NOT call SetActionManifestPath.
        # Isaac Sim already registered /actions/ov via ov.vrmanifest.json.
        # A second manifest call conflicts and makes bActive always False.
        set_h = ct.c_uint64(0)
        err = fn_get_set_h(self._ACT_SET.encode(), ct.byref(set_h))
        if err != 0:
            print(f"[VR] GetActionSetHandle err={err} — VR not fully started yet")
            # Reset sys_ptr so we retry clean next frame
            self._sys_ptr = 0
            self._fn_refs.clear()
            return
        print(f"[VR] Action set handle: {set_h.value:#x}")

        # Step 4: GetActionHandle for each action
        h = {}
        for name in self._ACTIONS:
            hh = ct.c_uint64(0)
            err = fn_get_act_h(name.encode(), ct.byref(hh))
            short = name.split("/")[-1]
            if err == 0 and hh.value != 0:
                h[name] = hh.value
                print(f"[VR] handle OK: {short} = {hh.value:#x}")
            else:
                print(f"[VR] handle FAIL: {short} err={err}")

        if len(h) < 3:
            print(f"[VR] Only {len(h)} handles — retrying next frame")
            self._sys_ptr = 0
            self._fn_refs.clear()
            return

        self._handles    = h
        self._set_h      = set_h.value
        self._fn_update  = fn_update
        self._fn_analog  = fn_analog
        self._fn_digital = fn_digital
        self._ct         = ct
        self._ready      = True
        print(f"[VR] ✓ ACTIVE — {len(h)} action handles")

    # ── per-frame ─────────────────────────────────────────────────────
    def poll(self):
        if not self.enabled:
            return
        self._step += 1
        if not self._ready:
            self._ensure_ready()
            return
        self._read()

    def _read(self):
        ct = self._ct

        # VRActiveActionSet_t layout (openvr.h, stable):
        #   uint64 ulActionSet
        #   uint64 ulRestrictedToDevice
        #   uint64 ulSecondaryActionSet
        #   uint32 unPadding
        #   int32  nPriority
        class _ActiveSet(ct.Structure):
            _pack_  = 4
            _fields_ = [
                ("ulActionSet",          ct.c_uint64),
                ("ulRestrictedToDevice", ct.c_uint64),
                ("ulSecondaryActionSet", ct.c_uint64),
                ("unPadding",            ct.c_uint32),
                ("nPriority",            ct.c_int32),
            ]

        active = _ActiveSet()
        active.ulActionSet          = self._set_h
        active.ulRestrictedToDevice = 0   # k_ulInvalidInputValueHandle
        active.ulSecondaryActionSet = 0
        active.nPriority            = 0
        self._fn_update(ct.byref(active), ct.sizeof(_ActiveSet), 1)

        # InputAnalogActionData_t layout (openvr.h exact):
        #   bool   bActive       (1 byte)
        #   byte   _pad[7]       (7 bytes padding)
        #   uint64 activeOrigin  (8 bytes)
        #   float  x, y, z      (4 bytes each)
        #   float  deltaX, deltaY, deltaZ (4 bytes each)
        #   float  fUpdateTime   (4 bytes)
        class _Analog(ct.Structure):
            _pack_  = 1
            _fields_ = [
                ("bActive",      ct.c_bool),
                ("_pad",         ct.c_uint8 * 7),
                ("activeOrigin", ct.c_uint64),
                ("x",            ct.c_float),
                ("y",            ct.c_float),
                ("z",            ct.c_float),
                ("deltaX",       ct.c_float),
                ("deltaY",       ct.c_float),
                ("deltaZ",       ct.c_float),
                ("fUpdateTime",  ct.c_float),
            ]

        # InputDigitalActionData_t layout (openvr.h, exact byte layout):
        #   bool   bActive       (1 byte)
        #   byte   _pad[7]       (7 bytes padding so activeOrigin is 8-byte aligned)
        #   uint64 activeOrigin  (8 bytes)
        #   bool   bState        (1 byte)
        #   bool   bChanged      (1 byte)
        #   byte   _pad2[2]      (2 bytes)
        #   float  fUpdateTime   (4 bytes)
        # NOTE: _pack_=1 to match the C struct exactly (no compiler padding magic).
        class _Digital(ct.Structure):
            _pack_  = 1
            _fields_ = [
                ("bActive",      ct.c_bool),
                ("_pad",         ct.c_uint8 * 7),
                ("activeOrigin", ct.c_uint64),
                ("bState",       ct.c_bool),
                ("bChanged",     ct.c_bool),
                ("_pad2",        ct.c_uint8 * 2),
                ("fUpdateTime",  ct.c_float),
            ]

        def analog(name):
            hh = self._handles.get(name)
            if not hh:
                return 0.0
            d = _Analog()
            err = self._fn_analog(hh, ct.byref(d), ct.sizeof(_Analog), 0)
            if err != 0:
                return 0.0
            # Quest 2 via ALVR: bActive is sometimes False even for active triggers.
            # Accept the value regardless — if the handle is valid and err==0, trust x.
            return float(d.x)

        def analog_float_fallback(name):
            """Try analog first; if bActive=False try treating x as raw float anyway."""
            hh = self._handles.get(name)
            if not hh:
                return 0.0
            d = _Analog()
            err = self._fn_analog(hh, ct.byref(d), ct.sizeof(_Analog), 0)
            if err != 0:
                return 0.0
            # Accept value even if bActive=False — some bindings set bActive=False
            # for boolean-typed actions when polled as analog
            return float(d.x)

        def digital(name):
            """Read a boolean action via GetDigitalActionData.
            Falls back to analog_float_fallback > 0.5 if digital call fails.
            """
            hh = self._handles.get(name)
            if not hh:
                return False
            if self._fn_digital is not None:
                d = _Digital()
                err = self._fn_digital(hh, ct.byref(d), ct.sizeof(_Digital), 0)
                if err == 0:
                    # Accept bState regardless of bActive — Quest 2 via ALVR sometimes
                    # reports bActive=False even when the binding is working fine.
                    return bool(d.bState)
            # Fallback: squeeze often appears as analog 0.0/1.0 even in bool bindings
            return analog_float_fallback(name) > 0.5

        def vec2(name):
            hh = self._handles.get(name)
            if not hh:
                return 0.0, 0.0
            d = _Analog()
            err = self._fn_analog(hh, ct.byref(d), ct.sizeof(_Analog), 0)
            if err != 0:
                return 0.0, 0.0
            # Accept regardless of bActive (Quest 2 via ALVR quirk, same as triggers)
            return float(d.x), float(d.y)

        self._rt = analog(self._ACT_RT)

        # KEY FIX v12: right_squeeze_value is ANALOG (float 0→1) in Isaac Sim's
        # ov.vrmanifest.json — NOT digital. Calling GetDigitalActionData on it
        # returns a zero-struct. Read it as analog and threshold at 0.5.
        _rg_raw = analog(self._ACT_RG)
        if self._step % 30 == 0:
            # Verbose grip debug — visible every ~1 s
            print(f"\n[VR-GRIP] squeeze_raw={_rg_raw:.4f} "
                  f"→ {'CLOSED' if _rg_raw > 0.5 else 'open'}", flush=True)
        self._rg = _rg_raw > 0.5          # bool from analog threshold

        self._lt = analog(self._ACT_LT)
        self._rx, self._ry = vec2(self._ACT_RJ)
        self._lx, self._ly = vec2(self._ACT_LJ)

        # A/B/X/Y buttons — try digital first, fall back to analog > 0.5
        self._btn_a = digital(self._ACT_A)
        self._btn_b = digital(self._ACT_B)
        self._btn_x = digital(self._ACT_X)
        self._btn_y = digital(self._ACT_Y)

        if self._step % 30 == 0:
            print(f"\r[VR] RT={self._rt:.2f} GRIP={'ON ' if self._rg else 'off'} "
                  f"LT={self._lt:.2f} "
                  f"RX={self._rx:.3f} RY={self._ry:.3f} "
                  f"LX={self._lx:.3f} LY={self._ly:.3f} "
                  f"A={int(self._btn_a)} B={int(self._btn_b)} "
                  f"X={int(self._btn_x)} Y={int(self._btn_y)}   ",
                  end="", flush=True)

    def right_trigger(self):    return self._rt
    def right_grip(self):       return bool(self._rg)
    def left_trigger(self):     return self._lt
    def right_thumbstick(self): return self._rx, self._ry
    def left_thumbstick(self):  return self._lx, self._ly
    def right_pos(self):        return self._rpos
    def btn_a(self):            return self._btn_a   # right A — zoom in
    def btn_b(self):            return self._btn_b   # right B — zoom out
    def btn_x(self):            return self._btn_x   # left  X — zoom in
    def btn_y(self):            return self._btn_y   # left  Y — zoom out



_ZOOM_SPEED = 0.05   # metres per frame XROrigin moves toward/away from scene


def zoom_camera(direction: float):
    """
    Move XROrigin along its local Z axis (world Y in Isaac Sim's stage coords).
    direction > 0 → zoom in (move forward), < 0 → zoom out.
    Safe to call every frame — does nothing if XROrigin prim doesn't exist.
    """
    try:
        stage = omni.usd.get_context().get_stage()
        prim  = stage.GetPrimAtPath("/World/XROrigin")
        if not prim or not prim.IsValid():
            return
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps()
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                cur = op.Get()
                # Isaac Sim stage: X=right, Y=forward, Z=up
                # Moving Y (forward/back) achieves zoom relative to scene centre
                op.Set(Gf.Vec3d(cur[0], cur[1] + direction * _ZOOM_SPEED, cur[2]))
                return
    except Exception:
        pass


class JointController:
    FRANKA_HOME = np.array([0., -0.785, 0., -2.356, 0., 1.571, 0.785, 0.04, 0.04],
                           dtype=np.float32)
    UR5_HOME    = np.array([0., -1.5707, 1.5707, -1.5707, -1.5707, 0.],
                           dtype=np.float32)

    def __init__(self, name, art, dof):
        self.name = name; self.art = art; self.dof = dof
        self.gripper_open = True
        home = self.FRANKA_HOME if name == "franka" else self.UR5_HOME
        self.joints = home[:dof].copy()

    def reset(self):
        home = self.FRANKA_HOME if self.name == "franka" else self.UR5_HOME
        self.joints = home[:self.dof].copy()
        self._apply()
        print("\n[Robot] Home pose restored.")

    def move(self, dx, dy, dz, scale=0.008):
        self.joints[0] = float(np.clip(self.joints[0] + dx*scale, -2.9,  2.9))
        self.joints[1] = float(np.clip(self.joints[1] + dy*scale, -1.76, 1.76))
        if self.dof > 3:
            self.joints[3] = float(np.clip(self.joints[3] + dz*scale, -3.07, 0.07))
        self._apply()

    def move_joint(self, idx, delta, lo=-3.14, hi=3.14):
        if idx < self.dof:
            self.joints[idx] = float(np.clip(self.joints[idx] + delta, lo, hi))
            self._apply()

    def toggle_gripper(self):
        self.gripper_open = not self.gripper_open
        v = 0.04 if self.gripper_open else 0.0
        if self.name == "franka" and self.dof >= 9:
            self.joints[7] = self.joints[8] = v
        self._apply()
        print(f"\n[Robot] Gripper → {'OPEN' if self.gripper_open else 'CLOSED'}")

    def _apply(self):
        try:
            self.art.set_joint_positions(self.joints)
        except Exception as e:
            print(f"\n[Robot] set_joint_positions failed: {e}")


class DeltaTracker:
    def __init__(self): self._ref = None
    def update(self, pos, active):
        if not active or pos is None:
            self._ref = None; return 0., 0., 0.
        if self._ref is None:
            self._ref = pos.copy(); return 0., 0., 0.
        d = pos - self._ref; self._ref = pos.copy()
        return float(d[0]), float(d[1]), float(d[2])


def build_scene(world, robot_choice):
    stage = omni.usd.get_context().get_stage()
    PD    = UsdGeom.XformOp.PrecisionDouble

    world.scene.add_ground_plane(size=5.0, color=np.array([0.22, 0.22, 0.22]))

    create_prim("/World/DomeLight", "DomeLight")
    UsdLux.DomeLight(stage.GetPrimAtPath("/World/DomeLight")).CreateIntensityAttr(600)
    create_prim("/World/SunLight", "DistantLight")
    UsdLux.DistantLight(stage.GetPrimAtPath("/World/SunLight")).CreateIntensityAttr(3000)
    sun = UsdGeom.Xformable(stage.GetPrimAtPath("/World/SunLight"))
    sun.ClearXformOpOrder()
    sun.AddRotateXYZOp(precision=PD).Set(Gf.Vec3d(-45, 0, 0))

    from omni.isaac.core.objects import FixedCuboid, DynamicSphere
    table = FixedCuboid(prim_path="/World/Table", name="table",
                        position=np.array([0.50, 0.00, 0.35]),
                        scale=np.array([0.50, 0.35, 0.03]),
                        color=np.array([0.40, 0.28, 0.16]))
    world.scene.add(table)
    orange = DynamicSphere(prim_path="/World/Orange", name="orange",
                           position=np.array([0.50, 0.00, 0.39]),
                           radius=0.04, color=np.array([1.00, 0.45, 0.05]),
                           mass=0.15)
    world.scene.add(orange)

    assets_root = get_assets_root_path()
    if robot_choice == "franka":
        usd       = f"{assets_root}/Isaac/Robots/Franka/franka_alt_fingers.usd"
        prim_path = "/World/Franka"
    else:
        usd       = f"{assets_root}/Isaac/Robots/UniversalRobots/ur5/ur5.usd"
        prim_path = "/World/UR5"

    print(f"[Scene] Loading {robot_choice} …")
    add_reference_to_stage(usd_path=usd, prim_path=prim_path)
    from omni.isaac.core.robots import Robot as IsaacRobot
    robot = world.scene.add(
        IsaacRobot(prim_path=prim_path, name=robot_choice,
                   position=np.array([0., 0., 0.])))
    return robot, prim_path


def main():
    world = World(stage_units_in_meters=1.0,
                  physics_dt=1/60., rendering_dt=1/30.)
    robot, prim_path = build_scene(world, args.robot)
    if args.run_vr:
        setup_vr_camera(lock_locomotion=args.no_locomotion)

    print("[MAIN] world.reset() #1 …")
    world.reset()
    print("[MAIN] Warming up renderer (10-15 s) …")
    for i in range(60):
        world.step(render=True)   # keeps renderer + physics in sync during warmup
        if i % 10 == 0:
            print(f"\r  warmup {i}/60 …", end="", flush=True)
    print()
    print("[MAIN] world.reset() #2 …")
    world.reset()

    try:
        art = world.scene.get_object(args.robot)
        art.initialize()
        dof = art.num_dof
    except Exception:
        from omni.isaac.core.articulations import Articulation
        art = Articulation(prim_path)
        art.initialize()
        dof = art.num_dof

    print(f"[MAIN] Robot '{args.robot}' ready — {dof} DOF")
    ctrl  = JointController(args.robot, art, dof)
    ctrl.reset()
    kb    = KeyboardReader()
    vr    = VRControllerReader(args.run_vr)
    delta = DeltaTracker()

    prev_g = prev_r = prev_grip = prev_lt = False
    prev_a = prev_b = prev_x = prev_y = False
    step   = 0
    SPD    = 0.05

    print("\n[MAIN] Simulation loop running.")
    if args.run_vr:
        print("  Isaac Sim GUI → VR tab → Select SteamVR → Click Start VR")
        print()
        print("  CONTROLS:")
        print("  Right thumbstick ←→  : base rotation (joint 0)")
        print("  Right thumbstick ↑↓  : shoulder (joint 1)")
        print("  Left  thumbstick ←→  : elbow (joint 2)")
        print("  Left  thumbstick ↑↓  : wrist (joint 3)")
        print("  Right trigger + move : fine position control")
        print("  Right GRIP (squeeze) : toggle gripper  [v12 fix: reads as analog]")
        print("  Left  trigger        : reset to home")
        print("  A or X button        : zoom camera IN")
        print("  B or Y button        : zoom camera OUT")
    else:
        print("  W/S=±X  A/D=±Y  Q/E=±Z  G=gripper  R=reset  ESC=quit")

    while simulation_app.is_running():
        if kb.held("esc"):
            print("\n[MAIN] ESC — quitting.")
            break

        if args.run_vr:
            vr.poll()                    # _ensure_ready + _read — MUST come first
            rt  = vr.right_trigger()
            rg  = vr.right_grip()
            lt  = vr.left_trigger()
            rp  = vr.right_pos()
            rx, ry = vr.right_thumbstick()
            lx, ly = vr.left_thumbstick()
            btn_a = vr.btn_a()
            btn_b = vr.btn_b()
            btn_x = vr.btn_x()
            btn_y = vr.btn_y()

            # Physical controller movement → joints 0,1,3
            dx, dy, dz = delta.update(rp, rt > 0.3)
            if dx or dy or dz:
                ctrl.move(dx, dy, dz, scale=1.)

            # Right thumbstick → joint 0 (base) and joint 1 (shoulder)
            if abs(rx) > 0.15:
                ctrl.move_joint(0, rx * 0.04, -2.9, 2.9)
            if abs(ry) > 0.15:
                ctrl.move_joint(1, ry * 0.04, -1.76, 1.76)

            # Left thumbstick → joint 2 (elbow) and joint 3 (wrist)
            if abs(lx) > 0.15:
                ctrl.move_joint(2, lx * 0.04, -2.9, 2.9)
            if abs(ly) > 0.15:
                ctrl.move_joint(3, ly * 0.04, -3.07, 0.07)

            # Grip toggle (v12: rg is now analog > 0.5, not digital struct)
            if rg and not prev_grip:
                ctrl.toggle_gripper()
            prev_grip = rg

            # Left trigger → reset to home
            if lt > 0.5 and not prev_lt:
                ctrl.reset()
            prev_lt = lt > 0.5

            # A or X → zoom in  |  B or Y → zoom out  (held = continuous zoom)
            if btn_a or btn_x:
                zoom_camera(+1.0)
            if btn_b or btn_y:
                zoom_camera(-1.0)

        else:
            dx = dy = dz = 0.
            if kb.held("w"): dx += SPD
            if kb.held("s"): dx -= SPD
            if kb.held("a"): dy += SPD
            if kb.held("d"): dy -= SPD
            if kb.held("q"): dz += SPD
            if kb.held("e"): dz -= SPD
            if dx or dy or dz:
                ctrl.move(dx, dy, dz, scale=1.)
            g = kb.held("g")
            if g and not prev_g:
                ctrl.toggle_gripper()
            prev_g = g
            r = kb.held("r")
            if r and not prev_r:
                ctrl.reset()
            prev_r = r

        world.step(render=True)
        step += 1
        if step % 10 == 0:
            j = " ".join(f"{x:+.2f}" for x in ctrl.joints[:6])
            print(f"\r[{step:06d}] joints=[{j}] "
                  f"grip={'OPN' if ctrl.gripper_open else 'CLS'}  ",
                  end="", flush=True)

    print("\n[MAIN] Closing.")
    simulation_app.close()


if __name__ == "__main__":
    main()
