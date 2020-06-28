import matplotlib.pyplot as plt
import numpy as np
from selfdrive.config import Conversions as CV

x_vel = [0.0, 4.166741231209736, 8.333258768790266, 12.500000000000002, 16.666741231209738, 20.833258768790266, 25.85853614889048, 30.52299570508232, 50.00000000000001, 70.0, 75.0, 80.0, 90.00000000000001]
y_dist_old = [1.3978, 1.4071, 1.4194, 1.4348, 1.4596, 1.4904, 1.5362, 1.5565, 1.5845, 1.6205, 1.6565, 1.6905, 1.7435]  # TRs
plt.plot(np.array(x_vel), y_dist_old, label='old roadtrip')


x = [0, 23.9, 55, 70, 91]
y = [1.0, 1.025, 1.1, 1.125, 1.06]
y_dist_new = [np.interp(x_, x, y) * y_ for x_, y_ in zip(x_vel, y_dist_old)]


# y_dist_new = [1.3978, 1.4071, 1.4194, 1.4348, 1.4596, 1.4904, 1.5362, 1.5565, 1.5845, 1.6205, 1.6565, 1.6905, 1.7435]
plt.plot(np.array(x_vel), y_dist_new, label='new roadtrip')

# y_dist = np.mean(np.array([y_dist_old, y_dist_new]).T, axis=1)
# plt.plot(np.array(x_vel) * CV.MS_TO_MPH, y_dist, label='avg. traffic')
# print(y_dist.tolist())
# ft = np.array(traffic_x_vel) * np.array(traffic_y_dist) * 3.28084
# print(ft.tolist())
# plt.plot(np.array(traffic_x_vel) * CV.MS_TO_MPH, ft, 'o-', label='traffic profile')
# plt.plot(np.array(x_vel) * CV.MS_TO_MPH, np.array(x_vel) * np.array(y_dist), 'o-', label='relaxed profile')


# plt.plot([min(x), max(x)], [0, 0], 'r--')
# plt.plot([0, 0], [min(y), max(y)], 'r--')

plt.xlabel('mph')
plt.ylabel('sec')

# poly = np.polyfit(x, y, 6)
# x = np.linspace(min(x), max(x), 100)
# y = np.polyval(poly, x)
# plt.plot(x, y, label='poly fit')

# to_round = True
# if to_round:
#   x = np.round(x, 4)
#   y = np.round(y, 5)
#   print('x = {}'.format(x.tolist()))
#   print('y = {}'.format(y.tolist()))

plt.legend()
plt.show()
