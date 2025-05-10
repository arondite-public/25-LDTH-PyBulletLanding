import numpy as np
from numpy.typing import NDArray


def compute_target_position(
    image: NDArray, state: NDArray, current_max_rpm: float
) -> tuple[NDArray, NDArray, float]:
    """Compute the target position for the drone from the provided RGB image

    Args:
        image (NDArray): An image captured by the drone's camera
        state (NDArray): The drone's state vector
        current_max_rpm (float): The current RPM of the drones rotors. You can set this to 0 once you think you've landed.

    Returns:
        NDArray: The target position for the drone
        NDArray: The target yaw, pitch and roll for the drone
        float: The new max_rpm for the drone.
    """

    ### IMPLEMENT YOUR SOLUTION HERE ###

    return (
        np.array([0, 0, -0.5]),
        np.array([0, 0, 0]),
        current_max_rpm,
    )  # Dummy position as demonstration
