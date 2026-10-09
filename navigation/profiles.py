"""Generic flight profiles; scene geometry and occupied-goal claims stay separate."""
from copy import deepcopy

SAFE5 = dict(
    navigation_mode='safe_route', safe_shortcuts=True, fast_history=True,
    route_buffer=.8, curve_accel_limit=2., flight_altitude=10.,
    v_mission=5., v_max=6., map_size=(60.,60.,32.), map_back_x=30.,
    res=.4, plan_res=.8, persistent_res=.8,
    persistent_map_path='data/maps/persistent_safe.npz',
    d_safe=3., d_min=1., drone_radius=.4, max_range=35., a_max=3.5,
    control_hz=30., control_dt=1./30., mapping_hz=4., planning_hz=3.,
    plan_pad=18., c_unknown=1.2,
    use_world_guide=True, guide_clearance=1.6, guide_altitude_max=160.,
    persistent_full_sensor=True, persistent_integrate_interval=2.,
    predictive_braking=True, progress_exploration=True,
    velocity_safety_guard=True,
    flight_altitude_min=5., flight_altitude_max=160., allow_safe_approach=True,
    approach_max_distance=35.,
)

def profile_overrides(name):
    if name=='legacy':return {}
    result=deepcopy(SAFE5)
    if name=='safe8':result.update(v_mission=8.,v_max=8.)
    elif name!='safe5':raise ValueError(f'Unknown profile: {name}')
    return result
