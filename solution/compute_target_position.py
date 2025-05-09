import numpy as np
from numpy.typing import NDArray


def compute_target_position(image: NDArray, state: NDArray) -> NDArray:
    """Compute the target position for the drone from the provided RGB image

    Args:
        image (NDArray): An image captured by the drone's camera
        state (NDArray): The drone's state vector

    Returns:
        NDArray: The target position for the drone
    """

    ### IMPLEMENT YOUR SOLUTION HERE ###

    return np.array([0, 0, 0])  # Dummy position as demonstration