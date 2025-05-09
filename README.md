# Arondite PyBullet Landing Challenge

## Setup

1. Install [uv](astral.sh/uv)
2. `uv sync`


## Running
The application can be run using the following command:

```bash
python -m drones.main.py
```
Note: This needs to be executed as a module from the root of the repository for imports to resolve correctly.

## Rules

You will need to implement the `compute_position` function within `drones/main.py`. This function has access to the drone state, alongside the latest image captured by the onboard camera.
