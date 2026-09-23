"""CLI bootstrap for polishing_v5.

SimulationApp must be created before importing modules that touch omni/isaac APIs.
"""
import argparse
import os
import sys
import traceback


def _strip_ros_paths():
    if "PYTHONPATH" in os.environ:
        os.environ["PYTHONPATH"] = ":".join(
            p for p in os.environ["PYTHONPATH"].split(":") if "/opt/ros" not in p
        )
    sys.path = [p for p in sys.path if "/opt/ros" not in p]


def _parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("obj_name_pos", nargs="?", help="Object name, for compatibility with positional calls")
    parser.add_argument("--obj_name", type=str, default=None)
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without opening the GUI")
    args, _ = parser.parse_known_args(argv)
    args.obj_name = (args.obj_name or args.obj_name_pos or "car").strip().lower()
    return args


def _enable_extension_safe(enable_extension, ext_name, required=False):
    """Enable a Kit extension without ever raising (headless runs must survive a missing ext)."""
    try:
        ok = bool(enable_extension(ext_name))
    except Exception as exc:  # noqa: BLE001
        ok = False
        print(f"[bootstrap] extension '{ext_name}' enable raised: {exc}", flush=True)
    if not ok:
        level = "ERROR" if required else "WARN"
        print(f"[bootstrap] [{level}] extension '{ext_name}' is not available", flush=True)
    return ok


def run_from_cli(argv=None):
    args = _parse_args(argv)
    _strip_ros_paths()

    from isaacsim import SimulationApp

    # 렌더러: Isaac Sim 6.0 의 SimulationApp 기본값은 RealTimePathTracing(무거움). 보기/녹화용은
    # RTX Real-Time(RaytracedLighting) 이 훨씬 가볍다. POLISH_RENDERER 로 교체
    # (RaytracedLighting | RealTimePathTracing | PathTracing | MinimalRendering).
    _renderer = os.environ.get("POLISH_RENDERER", "RaytracedLighting")
    simulation_app = SimulationApp({"headless": bool(args.headless), "renderer": _renderer})
    print(f"[bootstrap] renderer={_renderer} headless={bool(args.headless)}", flush=True)
    try:
        from isaacsim.core.utils.extensions import enable_extension

        # Isaac Sim 6: the ContactSensor wrapper (isaacsim.sensors.physics, extsDeprecated)
        # is not loaded by the default python experience; agent.py imports it at module level.
        _enable_extension_safe(enable_extension, "isaacsim.sensors.physics", required=True)

        # ROS 2 bridge is only used by the dashboard publisher (ros_publisher.py).
        # Skipped when POLISH_ROS_PUBLISH=0; never fatal when ROS is absent.
        if os.environ.get("POLISH_ROS_PUBLISH", "1") != "0":
            _enable_extension_safe(enable_extension, "isaacsim.ros2.bridge")
            try:
                import rclpy  # noqa: F401
                from std_msgs.msg import Float64  # noqa: F401
            except ImportError:
                pass

        from .runner import main

        main(simulation_app, obj_name=args.obj_name)
    except Exception:
        traceback.print_exc()
    finally:
        simulation_app.close()
