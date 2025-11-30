"""Controller implementations for followers."""
import numpy as np
from utils import wrap_angle


def follower_control(state, ref, neighbors_errors=None, my_neighbors=None, gains=None, k=1.0):
    x, y, theta = state
    x_ref, y_ref, theta_ref, theta_dot_ref, v_ref = ref

    # 1. Self Error (Global)
    ex_global = x_ref - x
    ey_global = y_ref - y

    # 2. Consensus Error (Global)
    fx_consensus_global = 0.0
    fy_consensus_global = 0.0

    # NEW: Variable to store sum of neighbor errors squared
    neighbors_sq_error_sum = 0.0

    if neighbors_errors and my_neighbors:
        for neighbor_id in my_neighbors:
            if neighbor_id in neighbors_errors:
                e_neighbor = neighbors_errors[neighbor_id]

                # Difference between MY global error and NEIGHBOR'S global error
                d_ex = ex_global - e_neighbor[0]
                d_ey = ey_global - e_neighbor[1]

                fx_consensus_global += d_ex
                fy_consensus_global += d_ey

                # NEW: Accumulate neighbor errors squared (||e_j||^2)
                neighbors_sq_error_sum += (e_neighbor[0]**2 + e_neighbor[1]**2)

    # 3. Apply Consensus Gain
    # Note: Using (+) as corrected in previous step
    ux_global = ex_global + (gains['consensus'] * fx_consensus_global)
    uy_global = ey_global + (gains['consensus'] * fy_consensus_global)

    # 4. Rotate Total "Virtual Error" to Local Frame
    c, s = np.cos(theta), np.sin(theta)

    ex_local =  ux_global * c + uy_global * s
    ey_local = -ux_global * s + uy_global * c

    # 5. Calculate Orientation Error
    e_theta = theta_ref - theta
    e_theta = (e_theta + np.pi) % (2 * np.pi) - np.pi

    # =========================================================
    # UNICYCLE CONTROLLER LAW
    # =========================================================

    # 1. Calculate Alpha (Damping based on error magnitude)
    # NEW: Added neighbors_sq_error_sum to the square root
    alpha = np.sqrt(k**2 + ex_local**2 + ey_local**2 + neighbors_sq_error_sum)

    # Feedforward
    v_ff = v_ref * np.cos(e_theta)
    w_ff = theta_dot_ref

    # Feedback
    v_cmd = v_ff + gains['cx'] * ex_local * v_ref

    # Sinc approximation
    if abs(e_theta) < 0.001:
        sinc_theta = 1.0
    else:
        sinc_theta = np.sin(e_theta) / e_theta

    # Feedback Rotation
    w_cmd = w_ff + (gains['cy'] * v_ref * ey_local * sinc_theta * (1/alpha)) + (gains['ct'] * e_theta)

    return v_cmd, w_cmd