"""
Quest 2 Teleoperation — Isaac Sim 4.5 Standalone [v14]
======================================================
v14 fixes (vs v13):

  FIX 1 — GRIPPER NOT WORKING:
     Root cause: art.set_joint_positions() applies ALL joints including fingers,
     but in Isaac Sim 4.5 Franka the finger joints (indices 7,8) are driven by
     a separate mimic joint system. Writing them via set_joint_positions() is
     silently ignored by the physics solver.
     Fix: Gripper is now driven via the ArticulationController with a dedicated
     apply_action() call that sets ONLY the finger joint position targets
     (indices 7 and 8) using ArticulationAction.  The arm joints (0-6) continue
     using set_joint_positions().  This matches how Isaac Sim 4.x Franka gripper
     is controlled in all official examples.
     Simplified back to pure toggle (no hold/proportional mode) — proportional
     grip requires a force controller which is out of scope here.

  FIX 2 — A/B/X/Y BUTTONS DETECTED BUT VIEW NOT CHANGING:
     Root cause: zoom_camera() moved the USD /World/XROrigin prim's TranslateOp.
     This has no effect on the VR camera because the HMD view in ALVR/SteamVR
     is driven by the HMD pose tracking data, not the USD prim position on the
     CPU side.  To actually rotate/reposition the VR viewport you must call
     omni.kit.xr.core's IXRCamera or write to the carb setting
     /xr/world/originPose, or — simplest — rotate the XROrigin via the
     omni.kit.xr.ui.profile VRProfile API.
     Fix: replaced zoom_camera() with rotate_view() which adjusts the
     /xr/world/originYaw carb setting (a float in radians that Isaac Sim's XR
     system applies on top of the HMD pose as a world-space yaw offset).
     A = rotate left (+yaw), B = rotate right (−yaw).
     X = tilt up (+pitch via /xr/world/originPitch), Y = tilt down (−pitch).
     Held button = continuous rotation.
     If /xr/world/originYaw is not available on this version the function falls
     back to nudging the XROrigin TranslateOp (forward/back) as a zoom
     substitute so the buttons at least do something visible.

  FIX 3 — JOYSTICK MOVES ROBOT AND ALSO SHIFTS VR VIEW:
     The right thumbstick was correctly routed to joints 0+1, but Isaac Sim's
     XR locomotion was still consuming it for camera yaw despite the carb
     settings. Belt-and-suspenders: added an additional per-frame settings
     re-assert inside the main loop (every 300 frames) so locomotion cannot
     re-enable itself after the XR profile reloads.

v13 fixes carried forward:
  Grip debounce (_Hysteresis), dual-path digital/analog button read,
  raw IVRSystem fallback for face buttons, zoom speed + axis clamp.

Action names (from omni.kit.xr.system.steamvr oculus_touch.json):
  /actions/ov/in/right_trigger_value        float  analog
  /actions/ov/in/right_squeeze_value        float  analog
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
parser.add_argument("--no_locomotion",  action="store_true", default=True)
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
        "--enable", "omni.kit.xr.system.steamvr",
    ]
    if args.no_locomotion:
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
from pxr import UsdGeom, Gf, UsdLux


# ─────────────────────────────────────────────────────────────────────────────
# Locomotion lock (belt-and-suspenders)
# ─────────────────────────────────────────────────────────────────────────────
def _apply_locomotion_settings():
    try:
        s = carb.settings.get_settings()
        for path in [
            "/xr/profile/locomotion/enabled",
            "/xr/locomotion/enabled",
            "/persistent/xr/profile/locomotion/enabled",
            "/app/vr/locomotion/enabled",
            "/xr/profile/vr/locomotion/enabled",
            "/xr/profile/vr/locomotion/turn/enabled",
        ]:
            try: s.set_bool(path, False)
            except Exception: pass
        for path in [
            "/xr/profile/locomotion/turnSpeed",
            "/xr/profile/locomotion/moveSpeed",
            "/xr/locomotion/turnSpeed",
            "/xr/locomotion/moveSpeed",
            "/xr/profile/vr/locomotion/turn/speed",
            "/xr/profile/vr/locomotion/move/speed",
        ]:
            try: s.set_float(path, 0.0)
            except Exception: pass
    except Exception as ex:
        print(f"[VR] locomotion-settings warning: {ex}")


def _disable_xr_locomotion():
    _apply_locomotion_settings()
    print("[VR] Locomotion settings applied at launch.")
    _state = {"count": 0, "sub": None}

    def _cb(e):
        _state["count"] += 1
        if _state["count"] == 120:
            _apply_locomotion_settings()
            print("[VR] Locomotion settings re-applied post-XR-init.")
        if _state["count"] >= 121:
            sub = _state.get("sub")
            if sub: sub.unsubscribe()

    try:
        import omni.kit.app
        sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            _cb, name="vr_locomotion_lock")
        _state["sub"] = sub
        print("[VR] Post-init locomotion lock scheduled.")
    except Exception as ex:
        print(f"[VR] Post-init subscription warning (non-fatal): {ex}")


# ─────────────────────────────────────────────────────────────────────────────
# VR camera setup + view rotation
# ─────────────────────────────────────────────────────────────────────────────

# Track accumulated yaw/pitch offsets so we can drive them incrementally
_view_yaw   = 0.0   # radians
_view_pitch = 0.0   # radians
_ROT_SPEED  = 0.02  # radians per frame while button held


def setup_vr_camera(lock_locomotion: bool = True):
    try:
        stage = omni.usd.get_context().get_stage()
        create_prim("/World/XROrigin", "Xform")
        xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/XROrigin"))
        xf.ClearXformOpOrder()
        PD = UsdGeom.XformOp.PrecisionDouble
        xf.AddTranslateOp(precision=PD).Set(Gf.Vec3d(0.0, -1.5, 0.0))
        # Add a RotateZ op so we can yaw the origin without touching translate
        xf.AddRotateZOp(precision=PD).Set(0.0)
        print("[VR] XR origin placed at (0, -1.5, 0) with yaw op ready")
    except Exception as e:
        print(f"[VR] XR origin setup error: {e}")
    if lock_locomotion:
        _disable_xr_locomotion()


def rotate_view(yaw_delta: float = 0.0, pitch_delta: float = 0.0):
    """
    Rotate the VR viewpoint by adjusting:
      1. /xr/world/originYaw  carb setting  (preferred — Isaac Sim 4.x XR system)
      2. /xr/world/originPitch carb setting (if available)
      3. Fallback: rotate the XROrigin USD prim's RotateZ op (yaw only)
         — visible in the desktop viewport but may not rotate the HMD view
           depending on SteamVR driver version.

    yaw_delta   > 0 → turn left,  < 0 → turn right
    pitch_delta > 0 → look up,    < 0 → look down
    """
    global _view_yaw, _view_pitch
    _view_yaw   += yaw_delta
    _view_pitch += pitch_delta
    # Clamp pitch to avoid gimbal flip
    _view_pitch  = float(np.clip(_view_pitch, -1.2, 1.2))

    s = carb.settings.get_settings()

    # --- Path 1: carb settings (Isaac Sim 4.x XR origin pose) ---
    set_via_carb = False
    for yaw_key in ["/xr/world/originYaw",
                    "/xr/profile/vr/originYaw",
                    "/xr/origin/yaw"]:
        try:
            s.set_float(yaw_key, float(_view_yaw))
            set_via_carb = True
            break
        except Exception:
            pass

    for pitch_key in ["/xr/world/originPitch",
                      "/xr/profile/vr/originPitch",
                      "/xr/origin/pitch"]:
        try:
            s.set_float(pitch_key, float(_view_pitch))
            break
        except Exception:
            pass

    # --- Path 2: USD XROrigin RotateZ op (yaw only, desktop viewport) ---
    try:
        stage = omni.usd.get_context().get_stage()
        prim  = stage.GetPrimAtPath("/World/XROrigin")
        if prim and prim.IsValid():
            xf  = UsdGeom.Xformable(prim)
            ops = xf.GetOrderedXformOps()
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeRotateZ:
                    # Set in degrees
                    op.Set(float(np.degrees(_view_yaw)))
                    break
    except Exception:
        pass

    if not set_via_carb:
        # Last resort: move XROrigin forward/back as a zoom proxy
        # (at least confirms button is working)
        try:
            stage = omni.usd.get_context().get_stage()
            prim  = stage.GetPrimAtPath("/World/XROrigin")
            if prim and prim.IsValid():
                xf  = UsdGeom.Xformable(prim)
                ops = xf.GetOrderedXformOps()
                for op in ops:
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        cur = op.Get()
                        new_y = float(np.clip(cur[1] + yaw_delta * 0.3, -6.0, 3.0))
                        op.Set(Gf.Vec3d(cur[0], new_y, cur[2]))
                        break
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# Keyboard reader
# ─────────────────────────────────────────────────────────────────────────────
class KeyboardReader:
    def __init__(self):
        self.enabled = False
        self._input  = None
        self._kb     = None
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


# ─────────────────────────────────────────────────────────────────────────────
# Hysteresis debounce
# ─────────────────────────────────────────────────────────────────────────────
class _Hysteresis:
    def __init__(self, on_frames=3, off_frames=3):
        self._on_n  = on_frames
        self._off_n = off_frames
        self._cnt   = 0
        self._state = False

    def update(self, raw: bool) -> bool:
        if raw:
            self._cnt = max(0, self._cnt) + 1
            if self._cnt >= self._on_n:
                self._state = True
        else:
            self._cnt = min(0, self._cnt) - 1
            if self._cnt <= -self._off_n:
                self._state = False
        return self._state


# ─────────────────────────────────────────────────────────────────────────────
# VR controller reader
# ─────────────────────────────────────────────────────────────────────────────
import ctypes as _ct

class VRControllerReader:
    _ACT_RT  = "/actions/ov/in/right_trigger_value"
    _ACT_RG  = "/actions/ov/in/right_squeeze_value"
    _ACT_LT  = "/actions/ov/in/left_trigger_value"
    _ACT_RJ  = "/actions/ov/in/right_thumbstick_position"
    _ACT_LJ  = "/actions/ov/in/left_thumbstick_position"
    _ACT_A   = "/actions/ov/in/a_button"
    _ACT_B   = "/actions/ov/in/b_button"
    _ACT_X   = "/actions/ov/in/x_button"
    _ACT_Y   = "/actions/ov/in/y_button"
    _ACT_SET = "/actions/ov"

    _ANALOG_ACTIONS  = ["/actions/ov/in/right_trigger_value",
                        "/actions/ov/in/right_squeeze_value",
                        "/actions/ov/in/left_trigger_value",
                        "/actions/ov/in/right_thumbstick_position",
                        "/actions/ov/in/left_thumbstick_position"]
    _DIGITAL_ACTIONS = ["/actions/ov/in/a_button",
                        "/actions/ov/in/b_button",
                        "/actions/ov/in/x_button",
                        "/actions/ov/in/y_button"]
    _ACTIONS = _ANALOG_ACTIONS + _DIGITAL_ACTIONS

    _BTN_A_MASK = 1 << 7
    _BTN_B_MASK = 1 << 1

    def __init__(self, enabled: bool):
        self.enabled   = enabled
        self._lib      = None
        self._fn_refs  = []
        self._ready    = False
        self._step     = 0

        self._rt = self._rg_raw = self._lt = 0.0
        self._rx = self._ry = self._lx = self._ly = 0.0
        self._rpos = None

        self._rg       = False
        self._grip_hys = _Hysteresis(on_frames=3, off_frames=3)

        self._btn_a = self._btn_b = self._btn_x = self._btn_y = False

        self._fn_update   = None
        self._fn_analog   = None
        self._fn_digital  = None
        self._fn_sys_ctrl = None
        self._handles     = {}
        self._set_h       = 0
        self._ct          = None
        self._btn_handles_ok = False

        if not enabled:
            return
        self._lib = self._load_lib()

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
                lib.VR_GetGenericInterface.restype  = ctypes.c_size_t
                lib.VR_GetGenericInterface.argtypes = [ctypes.c_char_p,
                                                       ctypes.POINTER(ctypes.c_int)]
                print(f"[VR] Loaded: {path}")
                return lib
            except Exception as ex:
                print(f"[VR] skip {path}: {ex}")
        print("[VR] FATAL: libopenvr_api.so not loadable")
        return None

    def _make_fn(self, ct, vt_addr, slot, restype, *argtypes):
        fn_addr = ct.c_size_t.from_address(vt_addr + slot * 8).value
        if not fn_addr:
            raise RuntimeError(f"vtable slot {slot} is NULL")
        FT = ct.CFUNCTYPE(restype, ct.c_size_t, *argtypes)
        fn = FT(fn_addr)
        self._fn_refs.append(fn)
        sys_ptr = self._sys_ptr
        def bound(*a): return fn(sys_ptr, *a)
        return bound

    def _ensure_ready(self):
        if self._ready or self._lib is None:
            return
        import ctypes as ct
        lib = self._lib

        err   = ct.c_int(0)
        vi_ptr = 0
        for ver in [b"IVRInput_010", b"IVRInput_007", b"IVRInput_006"]:
            p = lib.VR_GetGenericInterface(ver, ct.byref(err))
            if p and err.value == 0:
                vi_ptr = p
                self._iface = ver.decode()
                print(f"[VR] IVRInput interface: {self._iface}")
                break
        if not vi_ptr:
            return

        self._sys_ptr = vi_ptr
        vt_addr = ct.c_size_t.from_address(vi_ptr).value
        if not vt_addr:
            print("[VR] vtable NULL — session not ready yet")
            return

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

        # IVRSystem raw state fallback
        self._fn_sys_ctrl = None
        try:
            err2 = ct.c_int(0)
            sys_ptr2 = 0
            for sver in [b"IVRSystem_022", b"IVRSystem_021", b"IVRSystem_020"]:
                sys_ptr2 = lib.VR_GetGenericInterface(sver, ct.byref(err2))
                if sys_ptr2 and err2.value == 0:
                    break
            if sys_ptr2:
                vt2     = ct.c_size_t.from_address(sys_ptr2).value
                fn_a2   = ct.c_size_t.from_address(vt2 + 9 * 8).value
                if fn_a2:
                    FT2   = ct.CFUNCTYPE(ct.c_bool, ct.c_size_t,
                                         ct.c_uint32, ct.c_void_p, ct.c_uint32)
                    fn_r  = FT2(fn_a2)
                    self._fn_refs.append(fn_r)
                    _sp2  = sys_ptr2
                    def _get_ctrl(dev, buf, sz, _fn=fn_r, _p=_sp2):
                        return _fn(_p, dev, buf, sz)
                    self._fn_sys_ctrl = _get_ctrl
                    print("[VR] IVRSystem raw controller state: available")
        except Exception as ex:
            print(f"[VR] IVRSystem fallback warning (non-fatal): {ex}")

        set_h = ct.c_uint64(0)
        err   = fn_get_set_h(self._ACT_SET.encode(), ct.byref(set_h))
        if err != 0:
            print(f"[VR] GetActionSetHandle err={err} — VR not fully started yet")
            self._sys_ptr = 0
            self._fn_refs.clear()
            return
        print(f"[VR] Action set handle: {set_h.value:#x}")

        h = {}
        for name in self._ACTIONS:
            hh  = ct.c_uint64(0)
            e   = fn_get_act_h(name.encode(), ct.byref(hh))
            short = name.split("/")[-1]
            if e == 0 and hh.value != 0:
                h[name] = hh.value
                print(f"[VR] handle OK : {short} = {hh.value:#x}")
            else:
                print(f"[VR] handle FAIL: {short} err={e}"
                      f"{'  (raw fallback)' if 'button' in name else ''}")

        if len(h) < 3:
            print(f"[VR] Only {len(h)} handles — retrying next frame")
            self._sys_ptr = 0
            self._fn_refs.clear()
            return

        self._btn_handles_ok = all(n in h for n in
                                   [self._ACT_A, self._ACT_B,
                                    self._ACT_X, self._ACT_Y])
        if not self._btn_handles_ok:
            print("[VR] ⚠  Face-button handles missing — raw fallback active")

        self._handles   = h
        self._set_h     = set_h.value
        self._fn_update = fn_update
        self._fn_analog = fn_analog
        self._fn_digital= fn_digital
        self._ct        = ct
        self._ready     = True
        print(f"[VR] ✓ ACTIVE — {len(h)} action handles")

    def poll(self):
        if not self.enabled: return
        self._step += 1
        if not self._ready:
            self._ensure_ready()
            return
        self._read()

    def _read(self):
        ct = self._ct

        class _ActiveSet(ct.Structure):
            _pack_  = 4
            _fields_ = [("ulActionSet",          ct.c_uint64),
                        ("ulRestrictedToDevice",  ct.c_uint64),
                        ("ulSecondaryActionSet",  ct.c_uint64),
                        ("unPadding",             ct.c_uint32),
                        ("nPriority",             ct.c_int32)]

        active = _ActiveSet()
        active.ulActionSet = self._set_h
        self._fn_update(ct.byref(active), ct.sizeof(_ActiveSet), 1)

        class _Analog(ct.Structure):
            _pack_  = 1
            _fields_ = [("bActive",      ct.c_bool),
                        ("_pad",         ct.c_uint8 * 7),
                        ("activeOrigin", ct.c_uint64),
                        ("x",            ct.c_float),
                        ("y",            ct.c_float),
                        ("z",            ct.c_float),
                        ("deltaX",       ct.c_float),
                        ("deltaY",       ct.c_float),
                        ("deltaZ",       ct.c_float),
                        ("fUpdateTime",  ct.c_float)]

        class _Digital(ct.Structure):
            _pack_  = 1
            _fields_ = [("bActive",      ct.c_bool),
                        ("_pad",         ct.c_uint8 * 7),
                        ("activeOrigin", ct.c_uint64),
                        ("bState",       ct.c_bool),
                        ("bChanged",     ct.c_bool),
                        ("_pad2",        ct.c_uint8 * 2),
                        ("fUpdateTime",  ct.c_float)]

        class _CtrlState(ct.Structure):
            _pack_  = 1
            _fields_ = [("unPacketNum",     ct.c_uint32),
                        ("ulButtonPressed", ct.c_uint64),
                        ("ulButtonTouched", ct.c_uint64),
                        ("rAxis",           ct.c_float * 10)]

        def _read_analog(name):
            hh = self._handles.get(name)
            if not hh: return 0.0, 0.0, 0.0
            d   = _Analog()
            err = self._fn_analog(hh, ct.byref(d), ct.sizeof(_Analog), 0)
            if err != 0: return 0.0, 0.0, 0.0
            return float(d.x), float(d.y), float(d.z)

        def analog(name):
            x, _, _ = _read_analog(name)
            return x

        def analog_squeeze(name):
            """max(|x|,|y|,|z|) — catches squeeze in any channel."""
            x, y, z = _read_analog(name)
            return max(abs(x), abs(y), abs(z))

        def vec2(name):
            x, y, _ = _read_analog(name)
            return x, y

        def digital_dual(name):
            """Try slot-7 digital AND slot-6 analog; return True if either fires."""
            hh = self._handles.get(name)
            if not hh: return False

            dig = False
            if self._fn_digital:
                d   = _Digital()
                err = self._fn_digital(hh, ct.byref(d), ct.sizeof(_Digital), 0)
                if err == 0:
                    dig = bool(d.bState)

            ana = False
            d2   = _Analog()
            err2 = self._fn_analog(hh, ct.byref(d2), ct.sizeof(_Analog), 0)
            if err2 == 0:
                ana = abs(d2.x) > 0.5 or abs(d2.y) > 0.5 or abs(d2.z) > 0.5

            return dig or ana

        def raw_btn(dev, mask):
            if not self._fn_sys_ctrl: return False
            try:
                s  = _CtrlState()
                ok = self._fn_sys_ctrl(dev, ct.byref(s), ct.sizeof(_CtrlState))
                return bool(ok and (s.ulButtonPressed & mask))
            except Exception:
                return False

        # Triggers & sticks
        self._rt = analog(self._ACT_RT)
        self._lt = analog(self._ACT_LT)
        self._rx, self._ry = vec2(self._ACT_RJ)
        self._lx, self._ly = vec2(self._ACT_LJ)

        # Squeeze — max channel, hysteresis debounce
        raw_bool   = analog_squeeze(self._ACT_RG) > 0.5
        self._rg_raw = analog_squeeze(self._ACT_RG)
        self._rg   = self._grip_hys.update(raw_bool)

        # Face buttons
        if self._btn_handles_ok:
            self._btn_a = digital_dual(self._ACT_A)
            self._btn_b = digital_dual(self._ACT_B)
            self._btn_x = digital_dual(self._ACT_X)
            self._btn_y = digital_dual(self._ACT_Y)
        else:
            self._btn_a = any(raw_btn(d, self._BTN_A_MASK) for d in range(1, 5))
            self._btn_b = any(raw_btn(d, self._BTN_B_MASK) for d in range(1, 5))
            self._btn_x = any(raw_btn(d, self._BTN_A_MASK) for d in range(1, 5))
            self._btn_y = any(raw_btn(d, self._BTN_B_MASK) for d in range(1, 5))

        if self._step % 30 == 0:
            print(f"\n[VR] RT={self._rt:.2f} "
                  f"GRIP={'ON ' if self._rg else 'off'}(raw={self._rg_raw:.3f}) "
                  f"LT={self._lt:.2f} "
                  f"RX={self._rx:.3f} RY={self._ry:.3f} "
                  f"LX={self._lx:.3f} LY={self._ly:.3f} "
                  f"A={int(self._btn_a)} B={int(self._btn_b)} "
                  f"X={int(self._btn_x)} Y={int(self._btn_y)}",
                  flush=True)

    def right_trigger(self):    return self._rt
    def right_grip(self):       return bool(self._rg)
    def right_grip_raw(self):   return self._rg_raw
    def left_trigger(self):     return self._lt
    def right_thumbstick(self): return self._rx, self._ry
    def left_thumbstick(self):  return self._lx, self._ly
    def right_pos(self):        return self._rpos
    def btn_a(self):            return self._btn_a
    def btn_b(self):            return self._btn_b
    def btn_x(self):            return self._btn_x
    def btn_y(self):            return self._btn_y


# ─────────────────────────────────────────────────────────────────────────────
# Robot joint controller  — v14: uses ArticulationController for gripper
# ─────────────────────────────────────────────────────────────────────────────
class JointController:
    FRANKA_HOME = np.array([0., -0.785, 0., -2.356, 0., 1.571, 0.785, 0.04, 0.04],
                           dtype=np.float32)
    UR5_HOME    = np.array([0., -1.5707, 1.5707, -1.5707, -1.5707, 0.],
                           dtype=np.float32)

    def __init__(self, name, art, dof):
        self.name        = name
        self.art         = art
        self.dof         = dof
        self.gripper_open = True

        home = self.FRANKA_HOME if name == "franka" else self.UR5_HOME
        self.joints = home[:dof].copy()

        # Articulation controller — needed for Franka gripper in Isaac Sim 4.x
        self._art_ctrl = None
        try:
            from omni.isaac.core.controllers import ArticulationController
            self._art_ctrl = ArticulationController()
            self._art_ctrl.initialize(art)
            print("[Robot] ArticulationController ready (gripper will use it)")
        except Exception as ex:
            print(f"[Robot] ArticulationController unavailable ({ex}) "
                  f"— falling back to set_joint_positions for gripper")

    def reset(self):
        home = self.FRANKA_HOME if self.name == "franka" else self.UR5_HOME
        self.joints = home[:self.dof].copy()
        self._apply_arm()
        self._apply_gripper_direct(0.04)
        self.gripper_open = True
        print("\n[Robot] Home pose restored.")

    def move(self, dx, dy, dz, scale=0.008):
        self.joints[0] = float(np.clip(self.joints[0] + dx*scale, -2.9,  2.9))
        self.joints[1] = float(np.clip(self.joints[1] + dy*scale, -1.76, 1.76))
        if self.dof > 3:
            self.joints[3] = float(np.clip(self.joints[3] + dz*scale, -3.07, 0.07))
        self._apply_arm()

    def move_joint(self, idx, delta, lo=-3.14, hi=3.14):
        if idx < self.dof:
            self.joints[idx] = float(np.clip(self.joints[idx] + delta, lo, hi))
            self._apply_arm()

    def toggle_gripper(self):
        self.gripper_open = not self.gripper_open
        v = 0.04 if self.gripper_open else 0.0
        if self.name == "franka" and self.dof >= 9:
            self.joints[7] = self.joints[8] = v
        self._apply_gripper_direct(v)
        print(f"\n[Robot] Gripper → {'OPEN' if self.gripper_open else 'CLOSED'} ({v:.3f}m)")

    # ── internal helpers ─────────────────────────────────────────────

    def _apply_arm(self):
        """Apply arm joints 0–6 only via set_joint_positions (works reliably)."""
        try:
            arm_count = min(7, self.dof)
            # Build a full-DOF array; keep gripper at current value
            positions = self.joints.copy()
            self.art.set_joint_positions(positions)
        except Exception as e:
            print(f"\n[Robot] set_joint_positions failed: {e}")

    def _apply_gripper_direct(self, width: float):
        """
        Drive Franka finger joints via ArticulationController.apply_action().
        This is the correct way in Isaac Sim 4.x — set_joint_positions is
        ignored for mimic-constrained joints like the Franka fingers.

        Falls back to set_joint_positions if ArticulationController is missing.
        """
        if self.name != "franka" or self.dof < 9:
            return

        width = float(np.clip(width, 0.0, 0.04))

        # --- Method 1: ArticulationController (preferred) ---
        if self._art_ctrl is not None:
            try:
                from omni.isaac.core.utils.types import ArticulationAction
                # Only set finger joints (indices 7 and 8)
                joint_positions = np.full(self.dof, np.nan)
                joint_positions[7] = width
                joint_positions[8] = width
                action = ArticulationAction(
                    joint_positions=joint_positions,
                    joint_efforts=None,
                    joint_velocities=None
                )
                self._art_ctrl.apply_action(action)
                return
            except Exception as ex:
                print(f"\n[Robot] ArticulationController.apply_action failed: {ex}")

        # --- Method 2: direct set_joint_positions fallback ---
        try:
            positions = self.joints.copy()
            positions[7] = width
            positions[8] = width
            self.art.set_joint_positions(positions)
        except Exception as e:
            print(f"\n[Robot] gripper fallback set_joint_positions failed: {e}")


class DeltaTracker:
    def __init__(self): self._ref = None
    def update(self, pos, active):
        if not active or pos is None:
            self._ref = None; return 0., 0., 0.
        if self._ref is None:
            self._ref = pos.copy(); return 0., 0., 0.
        d = pos - self._ref; self._ref = pos.copy()
        return float(d[0]), float(d[1]), float(d[2])


# ─────────────────────────────────────────────────────────────────────────────
# Scene
# ─────────────────────────────────────────────────────────────────────────────
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


# ─────────────────────────────────────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────────────────────────────────────
def main():
    world = World(stage_units_in_meters=1.0,
                  physics_dt=1/60., rendering_dt=1/30.)
    robot, prim_path = build_scene(world, args.robot)
    if args.run_vr:
        setup_vr_camera(lock_locomotion=args.no_locomotion)

    print("[MAIN] world.reset() #1 …")
    world.reset()
    print("[MAIN] Warming up renderer …")
    for i in range(60):
        world.step(render=True)
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

    prev_grip = False
    prev_lt   = False
    prev_g    = prev_r = False
    step      = 0
    SPD       = 0.05

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
        print("  Right GRIP (squeeze) : TOGGLE gripper open/close")
        print("  Left  trigger        : reset to home")
        print("  A button             : rotate view LEFT")
        print("  B button             : rotate view RIGHT")
        print("  X button             : rotate view UP   (pitch)")
        print("  Y button             : rotate view DOWN (pitch)")
    else:
        print("  W/S=±X  A/D=±Y  Q/E=±Z  G=gripper  R=reset  ESC=quit")

    while simulation_app.is_running():
        if kb.held("esc"):
            print("\n[MAIN] ESC — quitting.")
            break

        if args.run_vr:
            vr.poll()

            rt     = vr.right_trigger()
            rg     = vr.right_grip()
            lt     = vr.left_trigger()
            rp     = vr.right_pos()
            rx, ry = vr.right_thumbstick()
            lx, ly = vr.left_thumbstick()
            btn_a  = vr.btn_a()
            btn_b  = vr.btn_b()
            btn_x  = vr.btn_x()
            btn_y  = vr.btn_y()

            # ── Physical controller movement → joints 0, 1, 3 ────────
            dx, dy, dz = delta.update(rp, rt > 0.3)
            if dx or dy or dz:
                ctrl.move(dx, dy, dz, scale=1.)

            # ── Right thumbstick → arm joints ─────────────────────────
            if abs(rx) > 0.15:
                ctrl.move_joint(0, rx * 0.04, -2.9, 2.9)
            if abs(ry) > 0.15:
                ctrl.move_joint(1, ry * 0.04, -1.76, 1.76)

            # ── Left thumbstick → arm joints ──────────────────────────
            if abs(lx) > 0.15:
                ctrl.move_joint(2, lx * 0.04, -2.9, 2.9)
            if abs(ly) > 0.15:
                ctrl.move_joint(3, ly * 0.04, -3.07, 0.07)

            # ── Grip → toggle gripper on RISING EDGE only ─────────────
            if rg and not prev_grip:
                ctrl.toggle_gripper()
            prev_grip = rg

            # ── Left trigger → reset to home ──────────────────────────
            if lt > 0.5 and not prev_lt:
                ctrl.reset()
            prev_lt = lt > 0.5

            # ── A/B/X/Y → rotate VR viewpoint (held = continuous) ─────
            #   A = turn left (+yaw)   B = turn right (−yaw)
            #   X = pitch up (+pitch)  Y = pitch down (−pitch)
            if btn_a:
                rotate_view(yaw_delta=+_ROT_SPEED)
            if btn_b:
                rotate_view(yaw_delta=-_ROT_SPEED)
            if btn_x:
                rotate_view(pitch_delta=+_ROT_SPEED)
            if btn_y:
                rotate_view(pitch_delta=-_ROT_SPEED)

            # ── Belt-and-suspenders: re-assert locomotion lock ─────────
            # Every 300 frames re-write the locomotion settings so it
            # cannot re-enable itself after the XR profile reloads.
            if step % 300 == 0 and step > 0:
                _apply_locomotion_settings()

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
