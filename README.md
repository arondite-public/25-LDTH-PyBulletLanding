# Arondite PyBullet Landing Challenge

## Problem

Conventional anti-submarine approaches rely on advanced underwater sensing as well as aerial reconnaissance using shipboard 
helicopters. When looking for a needle in a haystack, however, more eyes are always useful. A large fleet of autonomous systems to locate 
submarines, using Unmanned Aerial Vehicles (UAV) and Unmanned Surface Vessels (USV), is a novel approach being explored to 
enhance this capability. This approach, however, raises two challenges:
1. A USV will be impacted more by challenging sea-states given its smaller size than e.g. a frigate. 
2. We can also no-longer rely on the skills of naval pilots to bring the UAV back onto the deck, and without reliable and high-speed 
communication links, remote control is impractical.

Contestants will be asked to experiment with approaches for landing a UAV onto the deck of a USV. To help contestants, we assume the 
USV will have a set of fiduciary markers in the form of April Tags to help  orient the UAV and track the motion of the deck. 
We will use the PyBullet library to simulate the scenario, allowing us to vary the conditions and evaluate contestants’ 
approaches. Classical or deep learning-based approaches are permissible. It is up to contestants to decide how to orient your drone 
relative to the deck.

It is also up to contestants to decide how long the UAV takes to land, as speed and accuracy may need to be balanced. 
Contestants will be evaluated on the recovery rate over a number of trials in which the conditions become more and more challenging.

## Setup

1. Install [uv](https://github.com/astral-sh/uv). We recommend using `uv` as a package manager for this project!
2. `uv sync`

The application can then be run using the following command:

```bash
uv run python run_simulation.py
```
Note: This needs to be executed from the root of the repository for imports to resolve correctly.

This will open a window showing the simulation. If you'd like to run all the difficulties (more info on this below) in a row, you can run `bulk_run.sh` which will start the next difficulty every time you close the window.

### Solution

The only code you need to modify is in the `solution/` directory; please don't modify code elsewhere. The `compute_target_position` function is run in the main control loop - this is what you'll need to put your solution inside. If you want to modify something outside of this, please speak to the Arondite team!

## Evaluating

We will assess the recovery rate (the UAV's ability to land safely) in five different [sea states](https://en.wikipedia.org/wiki/Sea_state): SS0, SS2, SS3, SS5, SS7. SS0 is glassy and calm; SS7 is rough waters which typically only cargo ships and military vessels are able to operate in.

Whether or not a landing counts as succesful recovery will be assessed by the judges - if you land at 100km/h, or land upside down, this won't count! If you'd like clarification on whether or not a landing is succesful, please talk to the judges.