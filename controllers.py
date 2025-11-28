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
    # We want to pull our error towards the average of neighbors' errors
    fx_consensus_global = 0.0
    fy_consensus_global = 0.0

    if neighbors_errors and my_neighbors:
        for neighbor_id in my_neighbors:
            if neighbor_id in neighbors_errors:
                e_neighbor = neighbors_errors[neighbor_id]

                # Difference between MY global error and NEIGHBOR'S global error
                # If (My Error) > (Neighbor Error), I am "too far ahead" -> Slow down
                d_ex = ex_global - e_neighbor[0]
                d_ey = ey_global - e_neighbor[1]

                fx_consensus_global += d_ex
                fy_consensus_global += d_ey

    # 3. Apply Consensus Gain (Still in Global Frame)
    # Note: We SUBTRACT consensus. If my error is larger, I need to reduce my input.
    ux_global = ex_global - (gains['consensus'] * fx_consensus_global)
    uy_global = ey_global - (gains['consensus'] * fy_consensus_global)

    # 4. Rotate Total "Virtual Error" to Local Frame
    # Now we rotate the modified error vector into the robot's heading
    c, s = np.cos(theta), np.sin(theta)

    ex_local =  ux_global * c + uy_global * s
    ey_local = -ux_global * s + uy_global * c

    # 5. Calculate Orientation Error (Standard)
    # Normalize angle to [-pi, pi]
    e_theta = theta_ref - theta
    e_theta = (e_theta + np.pi) % (2 * np.pi) - np.pi

    # =========================================================
    # UNICYCLE CONTROLLER LAW
    # =========================================================
    # We now use 'ex_local' and 'ey_local' which include the consensus influence

    # Feedforward
    v_ff = v_ref * np.cos(e_theta)
    w_ff = theta_dot_ref

    # Feedback (Classic implementation)
    # v_cmd = v_ref * cos(e_theta) + Kx * ex_local
    v_cmd = v_ff + gains['cx'] * ex_local

    # w_cmd = w_ref + Ky * ey_local * sinc(e_theta) + Kt * sin(e_theta)
    # Sinc approximation: sin(x)/x. If x is small, ~1.
    if abs(e_theta) < 0.001:
        sinc_theta = 1.0
    else:
        sinc_theta = np.sin(e_theta) / e_theta

    w_cmd = w_ff + (gains['cy'] * ey_local * sinc_theta) + (gains['ct'] * np.sin(e_theta))

    return v_cmd, w_cmd