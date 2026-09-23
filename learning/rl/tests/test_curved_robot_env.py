"""스모크 — 원통 곡면 작업면에서 PhysX 접촉·품질 갱신 확인."""
import sys, os
import os as _os
sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))))
from isaaclab.app import AppLauncher
import argparse
p = argparse.ArgumentParser(); AppLauncher.add_app_launcher_args(p)
app = AppLauncher(p.parse_args()).app

import numpy as np, torch
from learning.rl.env.robot_polish_env import RobotPolishEnv
from learning.rl.env.robot_polish_env_cfg import RobotPolishEnvCfg

cfg = RobotPolishEnvCfg()
cfg.scene.num_envs = 2
cfg.surface_kind = "cylinder"
cfg.curvature_radius_m = 0.5
cfg.enable_pad_physical_contact = True
env = RobotPolishEnv(cfg, render_mode=None)
obs, _ = env.reset()
zero = torch.zeros(2, 2, device=env.device)
forces, faults = [], 0
for t in range(700):                       # 35 s — seek(~3s) + 폴리싱
    obs, r, term, trunc, _ = env.step(zero)
    forces.append(float(env._force_sensor_filt_n[0]))
    faults += int(env._sensor_fault.any())
f = np.array(forces[200:])
surf = env._surfaces[0]
print(f"[smoke] 접촉력(200스텝 이후): mean {f.mean():.3f}N  max {f.max():.3f}N  min {f.min():.3f}N")
print(f"[smoke] sensor fault 스텝: {faults}")
print(f"[smoke] 제거 발생: cumulative mean {surf.cumulative_removal_um.mean():.4f}um max {surf.cumulative_removal_um.max():.3f}um")
print(f"[smoke] 표면 곡면 확인: nominal z 범위 {surf.nominal_surface_xyz_m[...,2].min()*1000:.2f}~{surf.nominal_surface_xyz_m[...,2].max()*1000:.2f} mm")
ok = (2.0 < f.mean() < 12.0) and faults == 0 and surf.cumulative_removal_um.max() > 0.001
print(f"[smoke] RESULT={'PASS' if ok else 'FAIL'}")
env.close(); app.close()
sys.exit(0 if ok else 1)
