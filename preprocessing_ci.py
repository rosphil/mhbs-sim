"""Group the CI aCar GNSS samples into tours and resample them to 1 Hz simulation input.

Tour grouping follows ~/fleetcube/diss/preprocessing/tracks_ci.py: tracks shorter than
100 m are dropped, a tour starts with a departure from the Zatta base at least 15 min
after the previous departure and must end at the base, and tours idling for more than
24 h in total are discarded as broken recordings.

With --query, the raw inputs are first fetched again from the mobtrack database
using the queries in sql/.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import geopandas as gpd
import pandas as pd
import shapely
import sqlalchemy
import yaml


VEHICLES = [1700000969, 1700000970]  # aCars with GNSS samples in data_ci.pkl
THRESHOLD_TRACK_DIST = 100  # meters minimum covered to count as valid track - filters GNSS drift
THRESHOLD_IDLE_MAX = pd.Timedelta(hours=24)  # tours idling longer are seen as broken recordings
THRESHOLD_START_DELTA = pd.Timedelta(minutes=15)  # departures closer than this are grouped as one tour


def connect(credentials_path: Path) -> sqlalchemy.Engine:
    with open(credentials_path, "r") as file:
        credentials = yaml.safe_load(file)["sql_server"]
    url = sqlalchemy.engine.URL.create(
        drivername="postgresql+psycopg",
        host=credentials["host"],
        port=credentials["port"],
        username=credentials["user"],
        password=credentials["password"],
        database=credentials["database"],
    )
    return sqlalchemy.create_engine(url)


def run_query(con: sqlalchemy.Engine, sql_path: Path, output_path: Path, geometry: str | None = None) -> pd.DataFrame:
    """Run a stored query and pickle its result, optionally converting a WKB column to geometry."""
    result = pd.read_sql(sql=sql_path.read_text(), con=con)
    if geometry is not None:
        result[geometry] = result[geometry].apply(shapely.from_wkb)
        result = gpd.GeoDataFrame(result, geometry=geometry, crs="EPSG:4326")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    result.to_pickle(output_path)
    print(f"Wrote {len(result):,} rows of {sql_path.name} to {output_path}")
    return result


def load_tracks(path: Path, vehicles: list[int]) -> pd.DataFrame:
    """Load the raw trajectory query result and apply the diss track preprocessing."""
    tracks = pd.read_pickle(path)
    tracks = tracks[tracks["vehicle_id"].isin(vehicles)]
    tracks = tracks.rename(columns={
        "vehicle_id": "vehicle",
        "start_time": "time_start",
        "end_time": "time_end",
        "distance_track": "distance",
    })
    tracks = tracks[tracks["distance"] > THRESHOLD_TRACK_DIST]
    tracks = tracks[["vehicle", "time_start", "time_end", "distance", "modality", "start_at_base", "end_at_base"]]
    tracks = tracks.sort_values(["vehicle", "time_start"]).reset_index(drop=True)
    tracks["duration"] = tracks["time_end"] - tracks["time_start"]
    tracks["distance"] = tracks["distance"] / 1e3  # convert to km
    return tracks


def assign_tours(group: pd.DataFrame) -> pd.Series:
    """Return the tour number of each track of one vehicle, <NA> for tracks outside valid tours."""
    time_sufficient = group["time_start"].diff() > THRESHOLD_START_DELTA
    is_potential_start = time_sufficient & group["start_at_base"]
    tour_temp = is_potential_start.cumsum()

    # a potential tour is valid if its last track ends at the base
    is_last_in_tour = tour_temp != tour_temp.shift(-1)
    last_at_base = (is_last_in_tour & group["end_at_base"]).groupby(tour_temp).any()
    tour_valid = tour_temp.map(last_at_base)

    is_tour_start = is_potential_start & tour_valid
    tour = is_tour_start.cumsum().where(is_tour_start)
    return tour.ffill().where(tour_valid).astype("Int64")


def group_tours(tracks: pd.DataFrame, points: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Assign a global tourID to each track and summarize the valid tours."""
    tracks = tracks.copy()
    tracks["tour"] = pd.concat([assign_tours(group) for _, group in tracks.groupby("vehicle")])
    tracks = tracks.dropna(subset="tour")
    tracks["tourID"] = tracks.groupby(["vehicle", "tour"], sort=True).ngroup()

    # car tracks without any GNSS sample leave distance unaccounted for
    sampled = pd.MultiIndex.from_frame(points[["vehicle_id", "track_start_time"]].drop_duplicates())
    tracks["sampled"] = pd.MultiIndex.from_frame(tracks[["vehicle", "time_start"]]).isin(sampled)
    tracks["missing"] = (tracks["modality"] == "car") & ~tracks["sampled"]

    tours = tracks.groupby("tourID").agg(
        vehicle=("vehicle", "first"),
        time_start=("time_start", "min"),
        time_end=("time_end", "max"),
        distance=("distance", "sum"),
        time_active=("duration", "sum"),
        tracks=("distance", "size"),
        tracks_missing=("missing", "sum"),
    )
    tours["time_total"] = tours["time_end"] - tours["time_start"]
    tours["time_idle"] = tours["time_total"] - tours["time_active"]
    tours["speed_mean"] = 3600 * tours["distance"] / tours["time_active"].dt.total_seconds()
    tours = tours[tours["time_idle"] < THRESHOLD_IDLE_MAX]
    return tracks[tracks["tourID"].isin(tours.index)], tours


def resample_tour(points: pd.DataFrame, time_start: pd.Timestamp, time_end: pd.Timestamp) -> pd.DataFrame:
    """Average samples to 1 Hz over the whole tour; seconds without samples are standstill."""
    grid = pd.date_range(time_start.floor("s"), time_end.ceil("s"), freq="s")
    per_second = points.groupby(points["time"].dt.floor("s"))[["speed", "altitude"]].mean()
    trace = per_second.reindex(grid)
    trace["speed"] = trace["speed"].fillna(0.0)
    trace["altitude"] = trace["altitude"].interpolate(limit_area="inside").ffill().bfill()
    return pd.DataFrame({
        "time": (grid - grid[0]).total_seconds().to_numpy(),
        "datetime": grid,
        "speed": trace["speed"].to_numpy(),
        "altitude": trace["altitude"].to_numpy(),
    })


def build_timeseries(points: pd.DataFrame, tracks: pd.DataFrame, tours: pd.DataFrame) -> pd.DataFrame:
    """Return 1 Hz traces of all tours, identified by tourID."""
    points = points.merge(
        tracks[["vehicle", "time_start", "tourID"]],
        left_on=["vehicle_id", "track_start_time"],
        right_on=["vehicle", "time_start"],
    )
    traces = []
    for tour_id, tour_points in points.groupby("tourID", sort=True):
        tour = tours.loc[tour_id]
        trace = resample_tour(tour_points, tour["time_start"], tour["time_end"])
        trace.insert(0, "tourID", tour_id)
        trace.insert(1, "vehicle_id", tour["vehicle"])
        traces.append(trace)
    return pd.concat(traces, ignore_index=True)


def convert(tracks_path: Path, points_path: Path, output_path: Path, tours_path: Path, keep_incomplete: bool) -> pd.DataFrame:
    points = pd.DataFrame(pd.read_pickle(points_path)).drop(columns="geom")
    points = points[points["vehicle_id"].isin(VEHICLES)]
    tracks, tours = group_tours(load_tracks(tracks_path, VEHICLES), points)
    print(f"Grouped {len(tracks):,} tracks into {len(tours)} tours")

    if not keep_incomplete:
        incomplete = tours["tracks_missing"] > 0
        print(f"Dropping {incomplete.sum()} tours with car tracks lacking GNSS samples")
        tours = tours[~incomplete]
        tracks = tracks[tracks["tourID"].isin(tours.index)]

    timeseries = build_timeseries(points, tracks, tours)
    tours["distance_gnss"] = timeseries.groupby("tourID")["speed"].sum() / 1e3  # 1 s steps -> km

    output_path.parent.mkdir(parents=True, exist_ok=True)
    timeseries.to_pickle(output_path)
    tours.to_pickle(tours_path)
    print(f"Wrote {len(timeseries):,} rows of {len(tours)} tours to {output_path}")
    print(f"Wrote tour summary to {tours_path}")
    return timeseries


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--query", action="store_true",
                        help="Fetch --tracks and --points from the database before converting")
    parser.add_argument("--credentials", type=Path,
                        default=Path("~/fleetcube/diss/preprocessing/credentials.yaml").expanduser(),
                        help="YAML file with a 'sql_server' section (default: diss credentials)")
    parser.add_argument("--tracks", type=Path, default=Path("data/tracks_ci.pkl"),
                        help="Result of sql/tracks_ci.sql (default: data/tracks_ci.pkl)")
    parser.add_argument("--points", type=Path, default=Path("data/data_ci.pkl"),
                        help="Result of sql/speed_data_ci.sql (default: data/data_ci.pkl)")
    parser.add_argument("--output", type=Path, default=Path("data/tours_ci_1hz.pkl"),
                        help="1 Hz tour traces (default: data/tours_ci_1hz.pkl)")
    parser.add_argument("--tours", type=Path, default=Path("data/tours_ci_acar.pkl"),
                        help="Tour summary (default: data/tours_ci_acar.pkl)")
    parser.add_argument("--keep-incomplete", action="store_true",
                        help="Keep tours containing car tracks without GNSS samples")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.query:
        con = connect(args.credentials)
        run_query(con, Path("sql/tracks_ci.sql"), args.tracks)
        run_query(con, Path("sql/speed_data_ci.sql"), args.points, geometry="geom")
    convert(args.tracks, args.points, args.output, args.tours, args.keep_incomplete)
