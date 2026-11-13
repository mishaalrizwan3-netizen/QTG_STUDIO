"""A single fuel restoration plan built from measured primary tank channels."""
import math
PRIMARY = ('fuel_tank_left_main_level','fuel_tank_right_main_level','fuel_tank_center_level')
NAMES = ('left_main','right_main','center')

def valid_level(value):
    try: value=float(value)
    except (TypeError,ValueError): raise ValueError('Missing fuel level')
    if not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError('Invalid fuel level: '+str(value))
    return value

def primary_fuel_plan(state):
    levels=tuple(valid_level(state.get(key)) for key in PRIMARY)
    mass=state.get('total_fuel_weight_kg')
    if mass is not None and float(mass)>0.25 and not any(levels):
        raise ValueError('Manual reports fuel mass but all primary tanks are empty')
    all_levels=state.get('fuel_tank_levels_all') or {}
    conflicts=[]
    for name,level in zip(NAMES,levels):
        if name in all_levels:
            try:
                if abs(valid_level(all_levels[name])-level)>0.002:
                    conflicts.append(name)
            except ValueError: conflicts.append(name)
    # Never follow this primary write with a competing all-tank write.
    # This simulator's supported primary tanks are the exact recorded channels.
    return levels, conflicts

def fuel_mismatches(data,state,tolerance=0.002):
    levels,_=primary_fuel_plan(state)
    bad=[]
    for key,target in zip(PRIMARY,levels):
        try: value=valid_level(data.get(key))
        except ValueError:
            bad.append(key+' missing/invalid');continue
        if abs(value-target)>tolerance:
            bad.append(f'{key}: live={value:.6f}, Manual={target:.6f}')
    mass=state.get('total_fuel_weight_kg')
    if mass is not None:
        actual=data.get('total_fuel_quantity')
        try:
            actual=float(actual);target=float(mass)
            if not math.isfinite(actual) or abs(actual-target)>max(0.25,abs(target)*0.002):
                bad.append(f'total fuel kg: live={actual}, Manual={target}')
        except (ValueError,TypeError): bad.append('total fuel mass unavailable')
    return bad
