import sys
import os
sys.path.append(
 os.path.dirname(
  os.path.dirname(__file__)
 )
)

import numpy as np

from control.forward_kinematics import forward_kinematics


def test_zero_configuration():

    position = forward_kinematics(
        0,
        0,
        0
    )

    expected = np.array([0.37, 0])

    assert np.allclose(
        position,
        expected
    )
