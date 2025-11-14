"""Utility functions for the simulation."""
import numpy as np


def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


def unicycle_step(state, v, w, dt):
    """
    Integrate unicycle dynamics using RK4.
    
    Args:
        state: [x, y, theta]
        v: linear velocity
        w: angular velocity
        dt: time step
    
    Returns:
        new_state: [x, y, theta] after integration
    """
    k1_ = np.array([v * np.cos(state[2]), v * np.sin(state[2]), w])
    k2_ = np.array([v * np.cos(state[2] + 0.5 * dt * k1_[2]), 
                    v * np.sin(state[2] + 0.5 * dt * k1_[2]), w])
    k3_ = np.array([v * np.cos(state[2] + 0.5 * dt * k2_[2]), 
                    v * np.sin(state[2] + 0.5 * dt * k2_[2]), w])
    k4_ = np.array([v * np.cos(state[2] + dt * k3_[2]), 
                    v * np.sin(state[2] + dt * k3_[2]), w])
    new = state + (dt / 6.0) * (k1_ + 2 * k2_ + 2 * k3_ + k4_)
    new[2] = wrap_angle(new[2])
    return new
