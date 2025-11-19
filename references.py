"""Reference trajectory generation functions."""
import numpy as np


def vc_trajectory(t):
    """
    Compute virtual center trajectory and its derivatives.
    
    Args:
        t: time
    
    Returns:
        tuple: (x, y, theta, xd, yd, xdd, ydd, theta_dot, v)
    """
    r = 0.5
    omega = 0.5 * np.pi / 10.0  # One full circle in 10 seconds
    x = r * np.cos(omega * t)
    y = r * np.sin(omega * t)
    xd = -r * omega * np.sin(omega * t)
    yd = r * omega * np.cos(omega * t)
    xdd = -r * omega ** 2 * np.cos(omega * t)
    ydd = -r * omega ** 2 * np.sin(omega * t)
    theta = np.arctan2(yd, xd)
    theta_dot = (xd * ydd - yd * xdd) / (xd ** 2 + yd ** 2)
    v = np.sqrt(xd ** 2 + yd ** 2)
    return x, y, theta, xd, yd, xdd, ydd, theta_dot, v


def follower_reference(vc_state, P):
    """
    Compute follower reference from virtual center state and offset.
    
    Args:
        vc_state: tuple from vc_trajectory
        P: np.array([Px, Py]) offset in body frame
    
    Returns:
        tuple: (x_f, y_f, theta_f, theta_dot_f, v_f)
    """
    x_vc, y_vc, theta_vc, xd_vc, yd_vc, xdd_vc, ydd_vc, theta_dot_vc, v_vc = vc_state
    Px, Py = P
    
    # Position
    x_f = x_vc + Px * np.cos(theta_vc) - Py * np.sin(theta_vc)
    y_f = y_vc + Px * np.sin(theta_vc) + Py * np.cos(theta_vc)
    
    # Velocity
    xdot_f = xd_vc - Px * np.sin(theta_vc) * theta_dot_vc - Py * np.cos(theta_vc) * theta_dot_vc
    ydot_f = yd_vc + Px * np.cos(theta_vc) * theta_dot_vc - Py * np.sin(theta_vc) * theta_dot_vc
    
    theta_f = np.arctan2(ydot_f, xdot_f)
    v_f = np.sqrt(xdot_f ** 2 + ydot_f ** 2)
    
    # Acceleration
    xddot_f = xdd_vc - Px * (np.cos(theta_vc) * theta_dot_vc ** 2 + np.sin(theta_vc) * 0) - Py * (
                np.sin(theta_vc) * theta_dot_vc ** 2 - np.cos(theta_vc) * 0)
    yddot_f = ydd_vc - Px * (np.sin(theta_vc) * theta_dot_vc ** 2 - np.cos(theta_vc) * 0) + Py * (
                np.cos(theta_vc) * theta_dot_vc ** 2 + np.sin(theta_vc) * 0)
    
    theta_dot_f = (xdot_f * yddot_f - ydot_f * xddot_f) / (xdot_f ** 2 + ydot_f ** 2)
    
    return x_f, y_f, theta_f, theta_dot_f, v_f
