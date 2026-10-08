"""Convert raw ASC driving logs into one 1 Hz simulation input pickle."""

from __future__ import annotations

import argparse
from pathlib import Path

import can
import cantools
import numpy as np
import pandas as pd


BMS_MESSAGE_ID = 1537
SPEED_MESSAGE_ID = 395
OUTPUT_COLUMNS = ["trackID", "source_file", "time", "speed", "power", "BMS_SOC"]


def resample_1hz(trace: pd.DataFrame) -> pd.DataFrame:
    """Average a decoded trace over 1 s bins; time becomes the bin start in s."""
    bins = np.floor(trace["time"].to_numpy()).astype(int)
    resampled = trace.groupby(bins).agg(
        trackID=("trackID", "first"),
        source_file=("source_file", "first"),
        speed=("speed", "mean"),
        power=("power", "mean"),
        BMS_SOC=("BMS_SOC", "mean"),
    )
    resampled["time"] = resampled.index.astype(float)
    return resampled[OUTPUT_COLUMNS].reset_index(drop=True)


def convert_file(file_path: Path, database: cantools.database.Database, track_id: int) -> pd.DataFrame:
    """Read one ASC file and return its decoded BMS power trace."""
    bms_message = database.get_message_by_frame_id(BMS_MESSAGE_ID)
    speed_message = database.get_message_by_frame_id(SPEED_MESSAGE_ID)
    bms_rows: list[dict[str, float | int | str]] = []
    speed_rows: list[dict[str, float]] = []

    with can.io.ASCReader(str(file_path)) as asc_log:
        for message in asc_log:
            if message.arbitration_id == SPEED_MESSAGE_ID:
                signals = speed_message.decode(message.data)
                speed_rows.append(
                    {
                        "timestamp": message.timestamp,
                        "speed": signals["MCU1_VEHSPD"] / 3.6,
                    }
                )
                continue

            if message.arbitration_id != BMS_MESSAGE_ID:
                continue

            signals = bms_message.decode(message.data)
            bms_rows.append(
                {
                    "trackID": track_id,
                    "source_file": file_path.name,
                    "timestamp": message.timestamp,
                    "BMS_SOC": signals["BMS_SOC"],
                    "BMS_CURR": signals["BMS_CURR"],
                    "BMS_VOLT": signals["BMS_VOLT"],
                }
            )

    if not bms_rows:
        return pd.DataFrame(columns=OUTPUT_COLUMNS)

    result = pd.DataFrame(bms_rows)
    result["time"] = result["timestamp"] - result["timestamp"].iloc[0]
    result["power"] = -result["BMS_CURR"] * result["BMS_VOLT"]

    if speed_rows:
        speeds = pd.DataFrame(speed_rows).sort_values("timestamp")
        result = pd.merge_asof(
            result.sort_values("timestamp"),
            speeds,
            on="timestamp",
            direction="backward",
        )
        result["speed"] = result["speed"].bfill()
    else:
        result["speed"] = float("nan")

    return resample_1hz(result[OUTPUT_COLUMNS])


def convert_directory(input_directory: Path, dbc_path: Path, output_path: Path) -> pd.DataFrame:
    """Convert all ASC files in a directory and save one combined pickle."""
    database = cantools.database.load_file(str(dbc_path))
    input_files = sorted(
        file_path
        for file_path in input_directory.glob("*.asc")
        if "_RF_" in file_path.name
    )
    if not input_files:
        raise FileNotFoundError(f"No .asc files found in {input_directory}")

    converted = []
    for track_id, file_path in enumerate(input_files):
        print(f"[{track_id + 1}/{len(input_files)}] {file_path.name}")
        trace = convert_file(file_path, database, track_id)
        if not trace.empty:
            converted.append(trace)

    if not converted:
        raise ValueError("No BMS message data found in the ASC files")

    result = pd.concat(converted, ignore_index=True)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    result.to_pickle(output_path)
    print(f"Wrote {len(result):,} rows to {output_path}")
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-directory",
        type=Path,
        default=Path("data/de_raw"),
        help="Directory containing raw .asc files (default: data/de_raw)",
    )
    parser.add_argument(
        "--dbc",
        type=Path,
        default=Path("dbc/acar.dbc"),
        help="DBC file used to decode CAN messages (default: dbc/acar.dbc)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/data_de.pkl"),
        help="Output pickle path (default: data/data_de.pkl)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    convert_directory(args.input_directory, args.dbc, args.output)