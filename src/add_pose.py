
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    pose3 = initial_estimate.atPose2(X(3))
    new_x = 4.0 + math.sqrt(2)
    new_y = math.sqrt(2)
    new_theta = math.pi / 2
    new_pose = gtsam.Pose2(new_x, new_y, new_theta)

    from helperfunctions import add_pose_from_global
    graph, initial_estimate = add_pose_from_global(
        graph=graph,
        initial_estimate=initial_estimate,
        prev_key=X(3),
        new_key=X(4),
        prev_pose=pose3,
        new_pose_global=new_pose,
        odom_noise=ODOMETRY_NOISE,
    )

    return graph, initial_estimate
