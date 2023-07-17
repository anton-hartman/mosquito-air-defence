import random
import numpy as np
import matplotlib.pyplot as plt

from kalman_filter import KalmanFilter

# plt.ion()
plt.figure()

real_x = 0
real_vx = 0.5
real_y = 0
real_vy = 0.5
meas_variance = 0.1**2
sigma = np.sqrt(meas_variance)

kf = KalmanFilter(
    initial_x=0,
    initial_vx=1.0,
    x_accel_variance=0.1,
    initial_y=0,
    initial_vy=2.0,
    y_accel_variance=0.1,
)

DT = 0.1
NUM_STEPS = 1000
MEAS_EVERY_STEPS = 20

mus = []  # means = mu greek letter
covs = []
real_xs = []
real_vxs = []
real_ys = []
real_vys = []

for step in range(NUM_STEPS):
    # if step > 1:
        # real_vy += random.uniform(-5, 5)
        # real_vx += random.gauss(0, 0.5)
        # real_vx += random.uniform(-5, 5)

    covs.append(kf.cov)
    mus.append(kf.mean)

    ax = random.gauss(0, sigma)
    ay = random.gauss(0, sigma)

    real_x = real_x + (real_vx * DT) + (0.5 * ax * DT ** 2)
    real_y = real_y + (real_vy * DT) + (0.5 * ay * DT ** 2)
    real_vx = real_vx + ax * DT
    real_vy = real_vy + ay * DT

    kf.predict(dt=DT)
    meas_x = real_x + np.random.randn() * np.sqrt(meas_variance)
    meas_y = real_y + np.random.randn() * np.sqrt(meas_variance)
    meas_y = real_y
    # if step != 0 and step % MEAS_EVERY_STEPS == 0:
    #     kf.update(
    #         meas_x=meas_x,
    #         meas_y=meas_y,
    #         meas_ax_variance=meas_variance,
    #         meas_ay_variance=meas_variance,
    #     )

    real_xs.append(real_x)
    real_vxs.append(real_vx)
    real_ys.append(real_y)
    real_vys.append(real_vy)


plt.subplot(2, 2, 1)
plt.title("Position X")
plt.plot([mu[0] for mu in mus], "r")
plt.plot(real_xs, "b")
plt.plot([mu[0] - 2 * np.sqrt(cov[0, 0]) for mu, cov in zip(mus, covs)], "r--")
plt.plot([mu[0] + 2 * np.sqrt(cov[0, 0]) for mu, cov in zip(mus, covs)], "r--")

plt.subplot(2, 2, 3)
plt.title("Velocity X")
plt.plot(real_vxs, "b")
plt.plot([mu[1] for mu in mus], "r")
plt.plot([mu[1] - 2 * np.sqrt(cov[1, 1]) for mu, cov in zip(mus, covs)], "r--")
plt.plot([mu[1] + 2 * np.sqrt(cov[1, 1]) for mu, cov in zip(mus, covs)], "r--")

plt.subplot(2, 2, 2)
plt.title("Position Y")
plt.plot([mu[2] for mu in mus], "r")
plt.plot(real_xs, "b")
plt.plot([mu[2] - 2 * np.sqrt(cov[2, 2]) for mu, cov in zip(mus, covs)], "r--")
plt.plot([mu[2] + 2 * np.sqrt(cov[2, 2]) for mu, cov in zip(mus, covs)], "r--")

plt.subplot(2, 2, 4)
plt.title("Velocity Y")
plt.plot(real_vxs, "b")
plt.plot([mu[3] for mu in mus], "r")
plt.plot([mu[3] - 2 * np.sqrt(cov[3, 3]) for mu, cov in zip(mus, covs)], "r--")
plt.plot([mu[3] + 2 * np.sqrt(cov[3, 3]) for mu, cov in zip(mus, covs)], "r--")

plt.show()
# plt.ginput(1)
