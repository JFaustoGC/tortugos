"""Controller implementations for followers."""
import numpy as np
from utils import wrap_angle


def follower_control(state, ref, neighbors_errors=None, connectivity_row=None, 
                    gains=None, k=0):
    """
    Consensus-based formation controller for a follower.
    
    Args:
        state: [x, y, theta] current state
        ref: (x_ref, y_ref, theta_ref, w_ref, v_ref) reference
        neighbors_errors: array of neighbor errors, each in THEIR OWN body frame
        connectivity_row: connectivity vector for this follower
        gains: dict with 'cx', 'ct', 'cy', 'consensus' keys
        k: design parameter for alpha
    
    Returns:
        tuple: (v, w) control inputs
    """
    if gains is None:
        gains = {'cx': 0.8, 'ct': 10.0, 'cy': 1.2, 'consensus': 0.0}
    
    x, y, theta = state
    x_ref, y_ref, theta_ref, w_ref, v_ref = ref
    
    # Tracking error in body frame
    dx = x_ref - x
    dy = y_ref - y
    x_e = np.cos(theta) * dx + np.sin(theta) * dy
    y_e = -np.sin(theta) * dx + np.cos(theta) * dy
    theta_e = wrap_angle(theta_ref - theta)

    # Consensus terms
    x_consensus = 0.0
    y_consensus = 0.0
    x_consensus_squared = 0.0
    y_consensus_squared = 0.0

    if neighbors_errors is not None and connectivity_row is not None:
        for i in range(len(neighbors_errors)):
            if connectivity_row[i] == 0:
                continue

            x_en = neighbors_errors[i][0]
            y_en = neighbors_errors[i][1]


            dx = x_e - x_en
            dy = y_e - y_en
            
            # Linear consensus sums
            x_consensus += dx
            y_consensus += dy

            # Quadratic consensus sums
            x_consensus_squared += x_en * x_en
            y_consensus_squared += y_en * y_en

    alpha = np.sqrt(k ** 2 + x_e ** 2 + y_e ** 2 + x_consensus_squared + y_consensus_squared)

    # Control law
    consensus_gain = gains['consensus']
    v = v_ref * np.cos(theta_e) + gains['cx'] * (x_e + consensus_gain * x_consensus)
    w = (w_ref + gains['ct'] * theta_e + 
         gains['cy'] * v_ref * np.sinc(theta_e / np.pi) * (k / alpha) * 
         (y_e + consensus_gain * y_consensus))

    # Clip controls
    v = np.clip(v, -20.0, 20.0)
    w = np.clip(w, -3.0, 3.0)

    return v, w
