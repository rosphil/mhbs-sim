# mhbs-sim

Simulation of an external modular hybrid battery system (MHBS): an electric vehicle's fixed battery (fb) extended by portable battery packs (pb) connected through DC/DC converters. It estimates the range extension that 1–4 portable packs give on recorded real-world trips, and compares four energy-management strategies for splitting the demanded power between the fb and pb: `monotonous`, `proportional`, `mixed` and `full_power`.

## Repository layout

| Path | Contents |
| --- | --- |
| `components.py` | Vehicle longitudinal model, battery pack, DC/DC converter and energy manager |
| `simulation.py` | Time-step simulation of the fb alone (`base`) and of fb + pb (`mhbs_branch`) |
| `main.py` | SOC validation against measured trips, strategy comparison, result export |
| `preprocessing_de.py` | Decodes raw CAN logs (`.asc`) into 1 Hz trip data |
| `preprocessing_ci.py` | Groups GNSS samples into tours and resamples them to 1 Hz |
| `parameters/` | OCV / internal resistance curves (fb, pb) and DC/DC efficiency |
| `results/` | Simulation outputs: range extension, heatmap data, SOC validation |
| `controller/` | Arduino sketches for controlling the DC/DC converters over CAN |

## Usage

Requires Python ≥ 3.12 and [uv](https://docs.astral.sh/uv/).

```sh
uv sync
uv run preprocessing_de.py   # data/de_raw/*.asc -> data/data_de.pkl
uv run preprocessing_ci.py   # data/*_ci.pkl     -> data/tours_ci_1hz.pkl
uv run main.py               # writes results/*.csv
```

The input data (`data/`) and CAN databases (`dbc/`) are not part of this repository.

## License

Apache License 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
