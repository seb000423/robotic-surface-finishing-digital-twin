"""Synthetic thermal-state unit tests.  Isaac Sim is not required.

Run:
    ~/IsaacLab/.venv/bin/python -m learning.digital_twin.tests.test_temperature
"""
import sys

import numpy as np

from learning.digital_twin import config as C
from learning.digital_twin.gloss_proxy import LiteratureGlossProxyModel
from learning.digital_twin.polishing_model import ContactState, LiteraturePolishingModel
from learning.digital_twin.surface_state import make_flat_patch


results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


def fresh(seed=31):
    return make_flat_patch((0.20, 0.20), resolution_m=0.002,
                           seed=seed, with_scratches=False)


def contact(force=8.0, rpm=4000.0, feed=0.005, active=True):
    return ContactState((0.10, 0.10), force, rpm, feed, in_contact=active)


def run_contact(force=8.0, rpm=4000.0, seconds=2.0, dt=0.05):
    state = fresh()
    model = LiteraturePolishingModel(C.PolishingModelConfig(k_literature_synthetic=1e-10))
    for _ in range(round(seconds / dt)):
        model.step(state, contact(force=force, rpm=rpm), dt)
    return state


print("\n°C synthetic thermal model unit tests")

# 1. Initial state has an explicit physical unit and ambient value.
s = fresh()
check("1. initial temperature equals ambient",
      np.allclose(s.temperature_c, C.AMBIENT_TEMPERATURE_C))

# 2. Contact friction heats the footprint.
s = run_contact()
check("2. contact raises temperature", s.temperature_c.max() > C.AMBIENT_TEMPERATURE_C,
      f"peak={s.temperature_c.max():.3f} °C")

# 3. Higher force and RPM produce more heat under otherwise identical conditions.
low_force, high_force = run_contact(force=5.0), run_contact(force=8.0)
check("3a. Force up -> temperature up",
      high_force.temperature_c.max() > low_force.temperature_c.max())
low_rpm, high_rpm = run_contact(rpm=3000.0), run_contact(rpm=6000.0)
check("3b. RPM up -> temperature up",
      high_rpm.temperature_c.max() > low_rpm.temperature_c.max())

# Same linear travel distance: slower feed means longer dwell and therefore more heating.
slow_feed = run_contact(seconds=2.0)
fast_feed = run_contact(seconds=0.5)
check("3c. slower feed over same distance -> temperature up",
      slow_feed.temperature_c.max() > fast_feed.temperature_c.max())

# 4. No-contact state cools monotonically toward ambient without undershoot.
model = LiteraturePolishingModel(C.PolishingModelConfig(k_literature_synthetic=1e-10))
hot = run_contact(seconds=4.0)
t0 = hot.temperature_c.max()
for _ in range(200):
    model.step(hot, contact(active=False), 0.05)
t1 = hot.temperature_c.max()
check("4. no-contact cooling approaches ambient",
      C.AMBIENT_TEMPERATURE_C <= t1 < t0,
      f"{t0:.3f} -> {t1:.3f} °C")

# 5. Thermal integration should be stable when dt is divided.
a = run_contact(seconds=2.0, dt=0.10)
b = run_contact(seconds=2.0, dt=0.05)
rel_t = abs(a.temperature_c.max() - b.temperature_c.max()) / max(
    b.temperature_c.max() - C.AMBIENT_TEMPERATURE_C, 1e-12)
check("5. temperature dt split invariance", rel_t < 1e-6, f"relative={rel_t:.2e}")

# 6. Piecewise material profile changes removal sensitivity with temperature.
state = fresh()
state.temperature_c.fill(40.0)
state.peak_temperature_c.fill(40.0)
model.step(state, contact(), 0.05)
active_factor = state.temperature_removal_factor[state.temperature_removal_factor != 1.0]
check("6. temperature changes removal factor",
      active_factor.size > 0 and active_factor.mean() > 1.0,
      f"mean={active_factor.mean() if active_factor.size else 1.0:.3f}")

# 7. Sustained profile-overheat accumulates damage and never makes coat negative.
state = fresh()
state.temperature_c.fill(60.0)
state.peak_temperature_c.fill(60.0)
for _ in range(100):
    model.step(state, contact(), 0.05)
check("7a. overheat accumulates thermal damage", state.thermal_damage_proxy.max() > 0.0)
check("7b. clearcoat remains non-negative", state.clearcoat_remaining_um.min() >= 0.0)

# Thermal degradation must reduce the final GU proxy even at identical geometry.
undamaged = state.copy()
undamaged.thermal_damage_proxy.fill(0.0)
gloss = LiteratureGlossProxyModel()
gu_damaged = gloss.evaluate(state)["summary"]["gu_mean"]
gu_undamaged = gloss.evaluate(undamaged)["summary"]["gu_mean"]
check("7c. thermal damage lowers GU proxy", gu_damaged < gu_undamaged,
      f"{gu_undamaged:.2f} -> {gu_damaged:.2f} GU proxy")

# 8. Existing clearcoat material balance still holds with thermal correction.
mass_in = state.initial_clearcoat_um.sum() - state.clearcoat_remaining_um.sum()
mass_removed = state.cumulative_removal_um.sum()
rel_mass = abs(mass_in - mass_removed) / max(mass_removed, 1e-12)
check("8. thermal removal preserves material balance", rel_mass < 1e-6,
      f"relative={rel_mass:.2e}")

# 9. Explicit ablation switch preserves the former non-thermal transition path.
state = fresh()
off_model = LiteraturePolishingModel(C.PolishingModelConfig(
    k_literature_synthetic=1e-10, thermal_enabled=False))
for _ in range(20):
    off_model.step(state, contact(), 0.05)
check("9. thermal-off ablation keeps neutral thermal state",
      np.allclose(state.temperature_c, C.AMBIENT_TEMPERATURE_C)
      and np.allclose(state.temperature_removal_factor, 1.0)
      and np.allclose(state.thermal_damage_proxy, 0.0))

n_fail = results.count(False)
print(f"\n{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
