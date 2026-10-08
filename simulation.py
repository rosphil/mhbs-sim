import numpy as np
import pandas as pd
import components

CYCLES_MAX = 1000  # guard against profiles that never deplete the fb when repeated


def _drive(data: pd.DataFrame, repeat: bool):
    """Yield (time, row, cycle) along the profile; with repeat, it restarts seamlessly after one step."""
    time_start = float(data["time"].iloc[0])
    step_first = float(data["time"].iloc[1]) - time_start if len(data) > 1 else 1.0
    duration_cycle = float(data["time"].iloc[-1]) - time_start + step_first
    for cycle in range(CYCLES_MAX if repeat else 1):
        for row in data.itertuples(index=False):
            yield float(row.time) + cycle * duration_cycle, row, cycle
    if repeat:
        raise ValueError(f'Repeating the demand profile {CYCLES_MAX} times does not deplete the fb')


def base(
    data: pd.DataFrame,
    energy_nominal_fb: float,
    parameters_fb: dict[str, np.ndarray],
    soc_init_fb: float = 1,
    soc_min: float = 0.01,
    repeat: bool = False,
) -> pd.DataFrame:
    """Simulate a vehicle with only the fixed battery.

    The run ends when the fb reaches soc_min; with repeat, the demand profile is driven again until then.
    """

    fb = components.BatteryPack(
        name='fb',
        parameters=parameters_fb,
        energy_nominal_single=energy_nominal_fb,
        soc_init=soc_init_fb,
        soc_min=soc_min,
    )

    rows = []
    time_prev = float(data['time'].iloc[0])
    for time, row, cycle in _drive(data, repeat):
        dt = time - time_prev
        voltage, current, power, loss, soc = fb.step(power_req=float(row.power), dt=dt)

        result = {
            'time': time,
            'cycle': cycle,
            'power_dem': float(row.power),
            'voltage_fb': voltage,
            'current_fb': current,
            'power_fb': power,
            'loss_fb': loss,
            'soc_fb': soc,
        }
        if hasattr(row, 'speed'):
            result['speed'] = float(row.speed)
        if hasattr(row, 'BMS_SOC'):
            result['soc_fb_real'] = float(row.BMS_SOC) / 100.0
        rows.append(result)

        if fb.check_soc_min():
            break
        time_prev = time

    return pd.DataFrame(rows)


def mhbs_branch(
    data: pd.DataFrame,
    n_pb: int,
    strategy: str,
    energy_nominal_fb: float,
    energy_nominal_pb: float,
    parameters_fb: dict[str, np.ndarray],
    parameters_pb: dict[str, np.ndarray],
    parameters_dcdc: dict[str, np.ndarray],
    soc_init_fb: float = 1.0,
    soc_init_pb: float = 1.0,
    soc_min: float = 0.01,
    v_avg: float = 25.0,
    e_d: float = 280.0,
    repeat: bool = False,
) -> pd.DataFrame:
    """Simulate the fixed battery with portable batteries on a DCDC branch.

    The run ends when the fb reaches soc_min; with repeat, the demand profile is driven again until then.
    """

    fb = components.BatteryPack(
        name='fb',
        parameters=parameters_fb,
        energy_nominal_single=energy_nominal_fb,
        soc_init=soc_init_fb,
        soc_min=soc_min,
    )

    pb = components.BatteryPack(
        name='pb',
        parameters=parameters_pb,
        energy_nominal_single=energy_nominal_pb,
        packs_parallel=n_pb,
        soc_init=soc_init_pb,
        soc_min=soc_min,
    )

    dcdc = components.DCDCConverter(
        name='dcdc',
        parameters=parameters_dcdc,
    )

    ems = components.EnergyManager(
        strategy=strategy,
        v_avg=v_avg,
        e_d=e_d,
        )

    rows = []
    time_prev = float(data["time"].iloc[0])
    voltage_fb = fb.ocv

    for time, row, cycle in _drive(data, repeat):
        dt = time - time_prev

        power_dem = float(row.power)
        speed = float(row.speed)

        power_dem_branch = ems.step(
            power_dem=power_dem,
            fb=fb,
            pb=pb,
            speed=speed,
        )

        # converter output is bounded by its current limit at the fb voltage of the previous step
        power_dem_pb, power_out_dcdc, current_out_dcdc, efficiency_dcdc, loss_dcdc = dcdc.power_in(power_dem=power_dem_branch, voltage_out=voltage_fb)

        # the pb limits the converter input by its power capability and the energy left above soc_min
        power_max_pb = pb.power_max(dt=dt)
        if power_dem_pb > power_max_pb:
            power_out_low, power_out_high = 0.0, power_out_dcdc
            # bisection to 0.1 W, converter input rises monotonically with its output; nothing to search for an empty pb
            while power_max_pb > 0 and power_out_high - power_out_low > 0.1:
                power_out_mid = (power_out_low + power_out_high) / 2
                if dcdc.power_in(power_dem=power_out_mid, voltage_out=voltage_fb)[0] <= power_max_pb:
                    power_out_low = power_out_mid
                else:
                    power_out_high = power_out_mid
            power_dem_pb, power_out_dcdc, current_out_dcdc, efficiency_dcdc, loss_dcdc = dcdc.power_in(power_dem=power_out_low, voltage_out=voltage_fb)
        voltage_pb, current_pb, power_pb, loss_pb, soc_pb = pb.step(power_req=power_dem_pb, dt=dt)

        # fixed battery covers whatever the converter does not deliver
        voltage_fb, current_fb, power_fb, loss_fb, soc_fb = fb.step(power_req=power_dem - power_out_dcdc, dt=dt)

        rows.append({
            'time': time,
            'cycle': cycle,
            'speed': speed,
            'power_dem': power_dem,
            'voltage_fb': voltage_fb,
            'current_fb': current_fb,
            'power_fb': power_fb,
            'loss_fb': loss_fb,
            'soc_fb': soc_fb,
            'voltage_pb': voltage_pb,
            'current_pb': current_pb,
            'power_pb': power_pb,
            'loss_pb': loss_pb,
            'soc_pb': soc_pb,
            'power_out_dcdc': power_out_dcdc,
            'current_out_dcdc': current_out_dcdc,
            'efficiency_dcdc': efficiency_dcdc,
            'loss_dcdc': loss_dcdc,
        })

        if fb.check_soc_min():  # a depleted pb just stops delivering
            break

        time_prev = time

    return pd.DataFrame(rows)