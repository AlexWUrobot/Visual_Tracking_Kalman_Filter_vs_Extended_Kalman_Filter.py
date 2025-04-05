import numpy as np
import matplotlib.pyplot as plt

# Time parameters
dt = 0.1
t = np.arange(0, 20, dt)

# True target path (circular motion)
R = 10
omega = 0.2  # angular velocity
true_states = np.array([[R * np.cos(omega * ti), R * np.sin(omega * ti)] for ti in t])

# Measurements with noise
noise_std = 1.0
measurements = true_states + np.random.normal(0, noise_std, size=true_states.shape)

# ---------- Kalman Filter (Linear) ----------
# Assumes constant velocity model (linear system)
F = np.array([[1, 0],
              [0, 1]])  # Identity, because we assume position-only model (for simplicity)
H = np.array([[1, 0],
              [0, 1]])
              

Q = np.eye(2) * 0.01
R_kf = np.eye(2) * noise_std**2
P_kf = np.eye(2)
x_kf = np.array([0, 0])  # Initial state

kf_estimates = []

for z in measurements:
    # Prediction
    x_kf = F @ x_kf
    P_kf = F @ P_kf @ F.T + Q

    # Update
    y = z - H @ x_kf
    S = H @ P_kf @ H.T + R_kf
    K = P_kf @ H.T @ np.linalg.inv(S)
    x_kf = x_kf + K @ y
    P_kf = (np.eye(2) - K @ H) @ P_kf

    kf_estimates.append(x_kf.copy())

kf_estimates = np.array(kf_estimates)

# ---------- Extended Kalman Filter ----------
def h(x):
    """Nonlinear observation model: measures polar coordinates."""
    px, py = x
    r = np.sqrt(px**2 + py**2)
    theta = np.arctan2(py, px)
    return np.array([r, theta])

def H_jacobian(x):
    """Jacobian of h(x) with respect to x."""
    px, py = x
    r = np.sqrt(px**2 + py**2)
    if r < 1e-4:
        r = 1e-4
    H = np.array([
        [px / r, py / r],
        [-py / (r**2), px / (r**2)]
    ])
    return H

ekf_state = np.array([R, 0])  # start on x-axis
P_ekf = np.eye(2)
Q_ekf = np.eye(2) * 0.01
R_ekf = np.diag([noise_std**2, (np.deg2rad(10))**2])  # noise in r and theta
ekf_estimates = []

for z_true in true_states:
    # Simulate polar measurement from noisy Cartesian measurement
    noisy_z = z_true + np.random.normal(0, noise_std, size=2)
    r = np.linalg.norm(noisy_z)
    theta = np.arctan2(noisy_z[1], noisy_z[0])
    z = np.array([r, theta])

    # Prediction (assume static model for simplicity)
    x_pred = ekf_state
    P_pred = P_ekf + Q_ekf

    # Update
    H = H_jacobian(x_pred)
    y = z - h(x_pred)
    y[1] = (y[1] + np.pi) % (2 * np.pi) - np.pi  # Normalize angle

    S = H @ P_pred @ H.T + R_ekf
    K = P_pred @ H.T @ np.linalg.inv(S)
    ekf_state = x_pred + K @ y
    P_ekf = (np.eye(2) - K @ H) @ P_pred

    ekf_estimates.append(ekf_state.copy())

ekf_estimates = np.array(ekf_estimates)

# ---------- Plotting ----------
plt.figure(figsize=(10, 6))
plt.plot(true_states[:, 0], true_states[:, 1], 'k-', label='True Trajectory')
plt.plot(measurements[:, 0], measurements[:, 1], 'rx', alpha=0.5, label='Measurements')
plt.plot(kf_estimates[:, 0], kf_estimates[:, 1], 'b--', label='Kalman Filter')
plt.plot(ekf_estimates[:, 0], ekf_estimates[:, 1], 'g-', label='Extended Kalman Filter')
plt.legend()
plt.axis('equal')
plt.title("Tracking a Visual Target with KF and EKF")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid()
plt.show()


# ---------- Error Calculation ----------
def calc_error(estimate, true_val):
    return np.linalg.norm(estimate - true_val, axis=1)

error_measurements = calc_error(measurements, true_states)
error_kf = calc_error(kf_estimates, true_states)
error_ekf = calc_error(ekf_estimates, true_states)

# ---------- Error Plot ----------
plt.figure(figsize=(10, 5))
plt.plot(t, error_measurements, 'r-', label='Measurement Error')
plt.plot(t, error_kf, 'b--', label='KF Error')
plt.plot(t, error_ekf, 'g-', label='EKF Error')
plt.xlabel("Time [s]")
plt.ylabel("Position Error [Euclidean Distance]")
plt.title("Tracking Error Over Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


















# Why might KF/EKF errors be worse than measurements?

# 1. KF model is mismatched (wrong motion or observation assumptions)
# In your case, KF assumes a linear, static system (identity matrix F = I), but the true motion is circular and nonlinear.

# So KF is not actually doing useful prediction → it's smoothing noisy inputs in a poor model.

# 2. EKF update uses polar coordinates but measurements are Cartesian
# You're converting noisy Cartesian (x, y) to polar (r, θ), which may amplify the noise (especially near the origin).

# The EKF is estimating in Cartesian, but updating using noisy polar → the Jacobian and innovation mismatch can worsen the state estimate.

# 3. Poorly tuned noise covariances (Q and R)
# If process noise Q is too small → filter trusts its (bad) model too much.

# If measurement noise R is too large → filter doesn’t trust actual observations.

# Either of these makes the estimate diverge or lag behind.


# Improvement

# A 4D state KF and EKF (position + velocity)

# Linear measurement model (no polar conversion)

# Tuned Q and R

# Correct Jacobian for EKF (if using nonlinear motion)


import numpy as np
import matplotlib.pyplot as plt

# Time and simulation setup
dt = 0.1
t = np.arange(0, 20, dt)

# Ground truth: circular motion with constant speed
R = 10
omega = 0.2
true_states = np.array([
    [R * np.cos(omega * ti),
     R * np.sin(omega * ti),
     -R * omega * np.sin(omega * ti),
     R * omega * np.cos(omega * ti)] for ti in t
])

# Measurements: noisy (x, y)
noise_std = 1.0
measurements = true_states[:, 0:2] + np.random.normal(0, noise_std, size=(len(t), 2))

# System matrices for constant velocity model
F = np.array([[1, 0, dt, 0],
              [0, 1, 0, dt],
              [0, 0, 1,  0],
              [0, 0, 0,  1]])

H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])

Q = np.diag([0.01, 0.01, 0.1, 0.1])   # process noise
R_kf = np.eye(2) * noise_std**2      # measurement noise

# Initial estimates
x_kf = np.zeros(4)
P_kf = np.eye(4)
kf_estimates = []

x_ekf = np.zeros(4)
P_ekf = np.eye(4)
ekf_estimates = []

# EKF Jacobian function (linear here, but for completeness)
def jacobian_f(x):
    return F

def h(x):
    return H @ x

def jacobian_h(x):
    return H

# ---------- Filtering Loop ----------
for i in range(len(t)):
    z = measurements[i]

    # ----- Kalman Filter -----
    # Prediction
    x_kf = F @ x_kf
    P_kf = F @ P_kf @ F.T + Q

    # Update
    y = z - H @ x_kf
    S = H @ P_kf @ H.T + R_kf
    K = P_kf @ H.T @ np.linalg.inv(S)
    x_kf = x_kf + K @ y
    P_kf = (np.eye(4) - K @ H) @ P_kf
    kf_estimates.append(x_kf.copy())

    # ----- Extended Kalman Filter -----
    F_ekf = jacobian_f(x_ekf)
    H_ekf = jacobian_h(x_ekf)

    # Prediction
    x_pred = F_ekf @ x_ekf
    P_pred = F_ekf @ P_ekf @ F_ekf.T + Q

    # Update
    y = z - h(x_pred)
    S = H_ekf @ P_pred @ H_ekf.T + R_kf
    K = P_pred @ H_ekf.T @ np.linalg.inv(S)
    x_ekf = x_pred + K @ y
    P_ekf = (np.eye(4) - K @ H_ekf) @ P_pred
    ekf_estimates.append(x_ekf.copy())

kf_estimates = np.array(kf_estimates)
ekf_estimates = np.array(ekf_estimates)

# ---------- Error Computation ----------
def calc_error(estimates, true_vals):
    return np.linalg.norm(estimates[:, 0:2] - true_vals[:, 0:2], axis=1)

error_measurements = calc_error(measurements, true_states)
error_kf = calc_error(kf_estimates, true_states)
error_ekf = calc_error(ekf_estimates, true_states)

# ---------- Plots ----------
# Trajectory Plot
plt.figure(figsize=(10, 6))
plt.plot(true_states[:, 0], true_states[:, 1], 'k-', label='True Trajectory')
plt.plot(measurements[:, 0], measurements[:, 1], 'rx', alpha=0.4, label='Measurements')
plt.plot(kf_estimates[:, 0], kf_estimates[:, 1], 'b--', label='Kalman Filter')
plt.plot(ekf_estimates[:, 0], ekf_estimates[:, 1], 'yx', alpha=0.4, label='Extended Kalman Filter')
plt.axis('equal')
plt.title("Trajectory Tracking: KF vs EKF")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Error Plot
plt.figure(figsize=(10, 5))
plt.plot(t, error_measurements, 'r-', label='Measurement Error')
plt.plot(t, error_kf, 'b--', label='KF Error')
plt.plot(t, error_ekf, 'yx', label='EKF Error')
plt.xlabel("Time [s]")
plt.ylabel("Position Error [Euclidean Distance]")
plt.title("Tracking Error Over Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()