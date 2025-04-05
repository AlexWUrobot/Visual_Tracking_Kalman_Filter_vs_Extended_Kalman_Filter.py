# Visual Tracking,  Kalman Filter(KF) vs Extended Kalman Filter (EKF)

Here's a Python example that simulates tracking a visual target (e.g., moving object in 2D) using both a Kalman Filter (KF) and an Extended Kalman Filter (EKF). 
The target moves in 2D with nonlinear motion (e.g., circular motion), which makes it suitable for showing the advantage of EKF over a linear KF.

## Assumptions:

- The object moves in a circle: x = R * cos(ωt), y = R * sin(ωt)
- Measurement noise is Gaussian
- EKF handles the nonlinear observation model
