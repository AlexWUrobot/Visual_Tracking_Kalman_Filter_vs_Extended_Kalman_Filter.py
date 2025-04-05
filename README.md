# Visual Tracking,  Kalman Filter(KF) vs Extended Kalman Filter (EKF)

Here's a Python example that simulates tracking a visual target (e.g., moving object in 2D) using both a Kalman Filter (KF) and an Extended Kalman Filter (EKF). 
The target moves in 2D with nonlinear motion (e.g., circular motion), which makes it suitable for showing the advantage of EKF over a linear KF.

## Assumptions:

- The object moves in a circle: x = R * cos(ωt), y = R * sin(ωt)
- Measurement noise is Gaussian
- EKF handles the nonlinear observation model

![](case1.png)


![](case1_error.png)


## Why might KF/EKF errors be worse than measurements?
1.KF model is mismatched (wrong motion or observation assumptions)
2.EKF update uses polar coordinates but measurements are Cartesian
3.Poorly tuned noise covariances (Q and R)

![](case2.png)
![](case2_error.png)
