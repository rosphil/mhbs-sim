import multiprocessing
from functools import partial
from pathlib import Path
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import components
import simulation


TOUR_DISTANCE_MIN_KM = 10.0  # shorter tours than this threshold are not representative if repeated until empty
PB_COUNTS = (1, 2, 3, 4)  # numbers of portable packs compared against the fb alone


def get_battery_parameters(path: str) -> dict[str, np.ndarray]:
    parameters = pd.read_csv(path)
    return {
        "soc": parameters["soc"].to_numpy(),
        "ocv": parameters["ocv"].to_numpy(),
        "ri": parameters["ri"].to_numpy() / 1000.0  # conversion milliohm -> ohm
    }


def get_converter_parameters(path: str) -> dict[str, np.ndarray]:
    parameters = pd.read_csv(path)
    return {
        "current_out": parameters["current_out"].to_numpy(),
        "efficiency": parameters["efficiency"].to_numpy()
    }


def get_trip(data: pd.DataFrame, id: int) -> pd.DataFrame:
    if 'trackID' not in data.columns:
        raise ValueError("The loaded tracks file must contain a 'trackID' column.")
    tracks = data[data['trackID'] == id].copy()
    return tracks.reset_index(drop=True)


def prepare_trip_data(
    data: pd.DataFrame,
    vehicle: components.Vehicle | None = None,
    speed_window: int = 5,
) -> pd.DataFrame:
    """Normalize a raw trip before passing it to the component simulation."""
    if speed_window < 1:
        raise ValueError('speed_window must be positive')
    if not {'time', 'speed'}.issubset(data.columns):
        raise ValueError("Trip data must contain 'time' and 'speed' columns")

    vehicle = vehicle or components.Vehicle(m=2500, c_d=0.314, A_f=2.4, c_r=0.034)
    trip = data.copy().reset_index(drop=True)
    raw_time = trip['time']
    if pd.api.types.is_datetime64_any_dtype(raw_time):
        time = (raw_time - raw_time.iloc[0]).dt.total_seconds().to_numpy()
    else:
        time = raw_time.to_numpy(dtype=float)
        time = time - time[0]
    trip['time'] = time
    trip['speed'] = trip['speed'].astype(float).rolling(
        speed_window, center=True, min_periods=1
    ).mean()

    if 'power' not in trip:
        if len(trip) == 0:
            trip['power'] = pd.Series(dtype=float)
        elif {'voltage', 'current'}.issubset(trip.columns):
            trip['power'] = trip['voltage'] * trip['current']
        else:
            speed = trip['speed'].to_numpy()
            acceleration = np.gradient(speed, time) if len(trip) > 1 else np.zeros(1)
            if 'altitude' in trip:
                altitude = trip['altitude'].to_numpy(dtype=float)
                distance = np.concatenate(([0.0], np.cumsum(
                    np.maximum(speed[1:], 0.0) * np.diff(time)
                )))
                grade = np.degrees(np.arctan2(
                    np.gradient(altitude) if len(trip) > 1 else np.zeros(1),
                    np.maximum(np.gradient(distance), 1e-6) if len(trip) > 1 else np.ones(1),
                ))
            else:
                grade = np.zeros(len(trip))
            trip['power'] = [
                vehicle.power_total(v, alpha, accel)
                for v, alpha, accel in zip(speed, grade, acceleration)
            ]
    return trip


def trip_distance_km(data: pd.DataFrame) -> float:
    trip = prepare_trip_data(data)
    return float(np.trapezoid(trip['speed'], trip['time']) / 1000.0)


def trip_groups(data: pd.DataFrame):
    """Group data into trips: CI tours, DE recordings or raw CI tracks."""
    for id_columns in (['tourID'], ['trackID'], ['vehicle_id', 'track_start_time']):
        if set(id_columns).issubset(data.columns):
            keys = id_columns[0] if len(id_columns) == 1 else id_columns
            return data.groupby(keys, sort=True), id_columns
    raise ValueError('Cannot identify trips in supplied data')


def trip_distance_bins(
    data: pd.DataFrame,
    edges_km: np.ndarray | None = None,
) -> pd.DataFrame:
    if edges_km is None:
        edges_km = np.arange(0.0, 205.0, 5.0)
    groups, _ = trip_groups(data)

    distances = pd.Series({key: trip_distance_km(group) for key, group in groups})
    labels = [f'{left:g}-{right:g} km' for left, right in zip(edges_km[:-1], edges_km[1:])]
    bins = pd.cut(distances, bins=edges_km, labels=labels, right=False, include_lowest=True)
    counts = bins.value_counts(sort=False).rename('trip_count').reset_index()
    counts.columns = ['distance_bin', 'trip_count']
    return counts


def compare_strategies(
    data: pd.DataFrame,
    strategies: tuple[str, ...] = ('monotonous', 'proportional', 'mixed', 'full_power'),
    n_pbs: tuple[int, ...] = PB_COUNTS,
    vehicle: components.Vehicle | None = None,
    workers: int | None = None,
    **simulation_kwargs,
) -> pd.DataFrame:
    """Simulate the fb alone and every strategy with every pb count on every trip, over worker processes."""
    groups, id_columns = trip_groups(data)
    trip_ids, raw_trips = zip(*groups) if len(groups) else ((), ())
    simulate = partial(
        simulate_trip,
        id_columns=id_columns,
        strategies=strategies,
        n_pbs=n_pbs,
        vehicle=vehicle,
        simulation_kwargs=simulation_kwargs,
    )
    with multiprocessing.Pool(processes=workers) as pool:
        rows = [row for trip_rows in pool.starmap(simulate, zip(trip_ids, raw_trips)) for row in trip_rows]
    return pd.DataFrame(rows)


def simulate_trip(
    trip_id,
    raw_trip: pd.DataFrame,
    id_columns: list[str],
    strategies: tuple[str, ...],
    n_pbs: tuple[int, ...],
    vehicle: components.Vehicle | None,
    simulation_kwargs: dict,
) -> list[dict]:
    """Simulate one trip with the fb alone and all strategy/pb combinations; module level for worker processes.

    Range extension is the range gained over the fb alone, which runs as strategy 'fb_only' with n_pb 0.
    """
    trip = prepare_trip_data(raw_trip, vehicle=vehicle)
    distance = trip_distance_km(trip)
    base_kwargs = {
        key: value for key, value in simulation_kwargs.items()
        if key in ('energy_nominal_fb', 'parameters_fb', 'soc_init_fb', 'soc_min', 'repeat')
    }
    runs = [('fb_only', 0, simulation.base(data=trip, **base_kwargs))]
    runs += [
        (strategy, n_pb, simulation.mhbs_branch(data=trip, strategy=strategy, n_pb=n_pb, **simulation_kwargs))
        for n_pb in n_pbs
        for strategy in strategies
    ]

    identifiers = trip_id if isinstance(trip_id, tuple) else (trip_id,)
    range_fb = None
    rows = []
    for strategy, n_pb, result in runs:
        range_km = float(np.trapezoid(result['speed'], result['time']) / 1000.0)
        range_fb = range_km if range_fb is None else range_fb
        soc_fb_end = float(result['soc_fb'].iloc[-1])
        soc_pb_end = float(result['soc_pb'].iloc[-1]) if n_pb else np.nan
        energy_pb_end = soc_pb_end * simulation_kwargs['energy_nominal_pb'] * n_pb if n_pb else 0.0
        row = dict(zip(id_columns, identifiers))
        row.update({
            'strategy': strategy,
            'n_pb': n_pb,
            'distance_km': distance,
            'cycles': len(result) / len(trip),
            'range_km': range_km,
            'range_extension_km': range_km - range_fb,
            'range_extension_per_pb_km': (range_km - range_fb) / n_pb if n_pb else np.nan,
            'soc_fb_end': soc_fb_end,
            'soc_pb_end': soc_pb_end,
            'energy_remaining_wh': soc_fb_end * simulation_kwargs['energy_nominal_fb'] + energy_pb_end,
        })
        rows.append(row)
    return rows


def save_downsampled_soc(
    result: pd.DataFrame,
    output_path: str | Path,
    sample_period_s: float = 1.0,
) -> pd.DataFrame:
    """Save modeled and measured SOC on a regular time grid for plotting."""
    required_columns = {'time', 'soc_fb', 'soc_fb_real'}
    missing_columns = required_columns.difference(result.columns)
    if missing_columns:
        raise ValueError(f'Missing SOC columns: {sorted(missing_columns)}')
    if sample_period_s <= 0:
        raise ValueError('sample_period_s must be positive')

    source = result[['time', 'soc_fb', 'soc_fb_real']].dropna().sort_values('time')
    if source.empty:
        raise ValueError('No complete SOC samples available for export')

    time = source['time'].to_numpy(dtype=float)
    regular_time = np.arange(time[0], time[-1] + sample_period_s, sample_period_s)
    regular_time = regular_time[regular_time <= time[-1]]
    downsampled = pd.DataFrame({
        'time': regular_time,
        'soc_fb': np.interp(regular_time, time, source['soc_fb']),
        'soc_fb_real': np.interp(regular_time, time, source['soc_fb_real']),
    })

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    downsampled.to_csv(output_path, index=False, float_format='%.9g')
    return downsampled


VALIDATION_TRIPS = (21, 62, 90, 118)  # long DE trips for battery model validation


def soc_rmse_pct(result: pd.DataFrame) -> float:
    """Root mean square error of the simulated fb SOC against the measured BMS SOC in percentage points."""
    source = result[['soc_fb', 'soc_fb_real']].dropna()
    return float(np.sqrt(((source['soc_fb'] - source['soc_fb_real']) ** 2).mean()) * 100)


def validate_soc(
    data_de: pd.DataFrame,
    energy_nominal_fb: float,
    parameters_fb: dict[str, np.ndarray],
    output_directory: Path = Path('validation'),
) -> pd.DataFrame:
    """Simulate the validation trips with the fb alone, export simulated vs measured SOC and their RMSE."""
    scores = []
    for id in VALIDATION_TRIPS:
        trip = get_trip(data=data_de, id=id)
        result = simulation.base(
            data=trip,
            energy_nominal_fb=energy_nominal_fb,
            parameters_fb=parameters_fb,
        )
        save_downsampled_soc(result, output_directory / f'soc_track_{id:03d}.csv')
        scores.append({'trackID': id, 'rmse_pct': soc_rmse_pct(result)})
    scores = pd.DataFrame(scores)
    scores.to_csv(output_directory / 'soc_rmse.csv', index=False, float_format='%.3f')
    print(scores.to_string(index=False))
    return scores


def main():
    data_de = pd.read_pickle("data/data_de.pkl")
    data_ci = pd.read_pickle("data/tours_ci_1hz.pkl")
    tours_ci = pd.read_pickle("data/tours_ci_acar.pkl")

    parameters_fb = get_battery_parameters(path='parameters/fb.csv')
    parameters_pb = get_battery_parameters(path='parameters/pb.csv')
    parameters_dcdc = get_converter_parameters(path='parameters/dcdc.csv')

    energy_nominal_fb = 16500
    energy_nominal_pb = 2346
    vehicle_ci = components.Vehicle(m=2000, c_d=0.314, A_f=2.4, c_r=0.034)
    # monotonous strategy assumptions, fitted to the compared CI tours (> 45 km) with vehicle_ci:
    # mean speed while moving (>= 0.5 m/s) 34.8 km/h, net demand 258 Wh/km
    # the DE trips (measured power) match these as well: 36.4 km/h, 258 Wh/km
    v_avg_ci = 35.0  # km/h
    e_d_ci = 260.0  # Wh/km

    validate_soc(data_de, energy_nominal_fb=energy_nominal_fb, parameters_fb=parameters_fb)

    # CI tours use modeled power (vehicle_ci), DE trips their measured battery power;
    # tourID and trackID overlap, so trips are identified by (dataset, trip)
    distances_de = pd.Series({
        track_id: trip_distance_km(trip)
        for track_id, trip in data_de.groupby('trackID')
        if trip['speed'].notna().any()
    })
    datasets = {
        'ci': (data_ci, 'tourID', tours_ci['distance'], vehicle_ci),
        'de': (data_de, 'trackID', distances_de, None),
    }

    comparisons = []
    for dataset, (data, id_column, distances, vehicle) in datasets.items():
        distances = distances[distances > TOUR_DISTANCE_MIN_KM]
        result = compare_strategies(
            data[data[id_column].isin(distances.index)],
            vehicle=vehicle,
            v_avg=v_avg_ci,
            e_d=e_d_ci,
            repeat=True,
            energy_nominal_fb=energy_nominal_fb,
            energy_nominal_pb=energy_nominal_pb,
            parameters_fb=parameters_fb,
            parameters_pb=parameters_pb,
            parameters_dcdc=parameters_dcdc,
        )
        result = result.rename(columns={id_column: 'trip'})
        result.insert(0, 'dataset', dataset)
        result['trip_distance_km'] = result['trip'].map(distances)
        comparisons.append(result)
    comparison = pd.concat(comparisons, ignore_index=True).sort_values(['dataset', 'trip', 'n_pb', 'strategy'])
    output_path = Path('results') / 'range_extension.csv'
    output_path.parent.mkdir(parents=True, exist_ok=True)
    comparison.to_csv(output_path, index=False)

    extension = comparison[comparison['n_pb'] > 0]
    for metric in ('range_extension_km', 'range_extension_per_pb_km'):
        print(f'\n{metric}, mean over all tours')
        print(extension.pivot_table(index='n_pb', columns='strategy', values=metric).round(2).to_string())
        print(f'\n{metric}, mean per dataset')
        print(extension.pivot_table(index=['dataset', 'n_pb'], columns='strategy', values=metric).round(2).to_string())
    print('\nrange with the fb alone in km')
    print(comparison[comparison['n_pb'] == 0].groupby('dataset')['range_km'].describe().round(1).to_string())

    export_heatmap(comparison, Path('results') / 'range_extension_per_pb_heatmap.csv')


HEATMAP_STRATEGIES = ('full_power', 'monotonous', 'mixed', 'proportional')  # x axis order


def export_heatmap(comparison: pd.DataFrame, output_path: Path) -> pd.DataFrame:
    """Save the mean range extension per pb over all tours as a pgfplots matrix plot table.

    One row per cell in scanline order (strategy varies fastest), so it plots with mesh/cols=4.
    is_max marks the best strategy per n_pb and loss_pct the percentage lost against it.
    """
    extension = comparison[comparison['n_pb'] > 0]
    heatmap = extension.groupby(['n_pb', 'strategy'])['range_extension_per_pb_km'].mean().reset_index()
    heatmap['col_index'] = heatmap['strategy'].map({name: index for index, name in enumerate(HEATMAP_STRATEGIES)})
    heatmap['row_index'] = heatmap['n_pb'].rank(method='dense').astype(int) - 1
    best = heatmap.groupby('n_pb')['range_extension_per_pb_km'].transform('max')
    heatmap['is_max'] = (heatmap['range_extension_per_pb_km'] == best).astype(int)
    heatmap['loss_pct'] = (100 * (best - heatmap['range_extension_per_pb_km']) / best).round(1)
    heatmap['range_extension_per_pb_km'] = heatmap['range_extension_per_pb_km'].round(3)
    heatmap = heatmap.sort_values(['row_index', 'col_index'])[
        ['n_pb', 'strategy', 'range_extension_per_pb_km', 'col_index', 'row_index', 'is_max', 'loss_pct']
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    heatmap.to_csv(output_path, index=False)
    return heatmap

if __name__ == '__main__':
    main()
