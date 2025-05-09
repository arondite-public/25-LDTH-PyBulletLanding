import time
from dataclasses import dataclass
from itertools import product
from typing import List, Optional

import numpy as np
import pybullet as p
from numpy.typing import NDArray


@dataclass
class WaveData:
    wl: float
    amp: float
    ws: float
    theta: float

    @property
    def dirvec(self) -> NDArray[np.single]:
        return np.array([np.sin(self.theta), np.cos(self.theta)])


def eval_wave(
    *, wd: WaveData, timestep: float, coord: NDArray[np.single]
) -> NDArray[np.single]:
    return wd.amp * np.sin(wd.wl * (np.dot(coord, wd.dirvec) + wd.ws * timestep))


def makewave_random(seed: Optional[int] = None) -> WaveData:
    if seed:
        np.random.seed(seed)
    # wavelength from 2-10m
    wavelength = 2 + 8 * np.random.rand()
    # amplitude from 0 - 1.5m
    amplitude = 0.8 * np.random.rand()
    # wavespeed from 0.1-2m/s
    wavespeed = 0.1 + 1.9 * np.random.random()
    # theta from [-pi,pi)
    theta = -np.pi + 2 * np.pi * np.random.random()
    return WaveData(wl=wavelength, amp=amplitude, ws=wavespeed, theta=theta)


def makewave_easy() -> List[WaveData]:
    return [
        WaveData(wl=6, amp=0.6, ws=0.8, theta=0),
        WaveData(wl=5, amp=0.4, ws=0.4, theta=1.9),
    ]


def restoration_forces_easy(objIdx: int):
    # Motor position control with limited force gives is a restoration force back to neutral position.
    # Up-down restoration force
    p.setJointMotorControl2(objIdx, 0, p.POSITION_CONTROL, targetPosition=0, force=350)
    # Y rotation restoration Torque
    p.setJointMotorControl2(objIdx, 1, p.POSITION_CONTROL, targetPosition=0, force=10)
    # X rotation restoration Torque
    p.setJointMotorControl2(objIdx, 2, p.POSITION_CONTROL, targetPosition=0, force=10)


def apply_waveforce_to_platform(
    timestep: float, waves: List[WaveData], platformObjectIdx: int
):
    bound, samples = 0.2, 3
    disp = np.zeros([samples, samples], dtype=np.single)
    x = np.linspace(start=-bound, stop=bound, num=samples, dtype=np.single)
    y = np.linspace(start=-bound, stop=bound, num=samples, dtype=np.single)
    xx, yy = np.meshgrid(x, y)
    coord = np.stack([xx, yy], axis=-1)
    for wave in waves:
        disp += eval_wave(wd=wave, timestep=timestep, coord=coord)
    force_vec = disp * 20.0  # Spring(esque) constant
    for idx, jdx in product(range(samples), range(samples)):
        p.applyExternalForce(
            platformObjectIdx,
            1,
            (0.0, 0.0, force_vec[idx, jdx].item()),
            (x[idx], y[jdx], 0.0),
            p.LINK_FRAME,
        )
        p.applyExternalForce(
            platformObjectIdx,
            2,
            (0.0, 0.0, force_vec[idx, jdx].item()),
            (x[idx], y[jdx], 0.0),
            p.LINK_FRAME,
        )


if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)
    res = p.loadURDF(
        "arondite/assets/platform.urdf", physicsClientId=physicsClient, useFixedBase=1
    )

    p.setGravity(0, 0, -9.81)
    waves = makewave_easy()
    restoration_forces_easy(res)
    t = 0.0
    for i in range(1000):
        if (
            i < 200
        ):  # Only apply wave forces for first 200 steps to test restoration force.
            apply_waveforce_to_platform(timestep=t, waves=waves, platformObjectIdx=0)
        p.stepSimulation()
        t += 1.0 / 60.0
        time.sleep(1.0 / 60.0)
