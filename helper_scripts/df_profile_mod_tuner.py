from numpy import interp


def to_mph(x):
  return [i * 2.23694 for i in x]


def to_ms(x):
  return [i * 0.44704 for i in x]


p_mod_x = [5., 30., 55., 80.]
for v_ego in p_mod_x:
  if v_ego != 80.:
    continue
  # roadtrip
  x_vel = [0.0, 1.8627, 3.7253, 5.588, 7.4507, 9.3133, 11.5598, 13.645, 22.352, 31.2928, 33.528, 35.7632, 40.2336]
  x_vel = to_mph(x_vel)
  y_dist = [1.3978, 1.4132, 1.4318, 1.4536, 1.485, 1.5229, 1.5819, 1.6203, 1.7238, 1.8231, 1.8379, 1.8495, 1.8535]  # avg. 7.3 ft closer from 18 to 90 mph

  TR_traffic = interp(v_ego, x_vel, y_dist)

  traffic_mod_pos = [0.92, 0.7, 0.25, 0.15]
  traffic_mod_neg = [1.1, 1.3, 2.0, 2.3]

  traffic_mod_pos = interp(v_ego, p_mod_x, traffic_mod_pos)
  traffic_mod_neg = interp(v_ego, p_mod_x, traffic_mod_neg)

  # relaxed
  x_vel = [0.0, 1.8627, 3.7253, 5.588, 7.4507, 9.3133, 11.5598, 13.645, 22.352, 31.2928, 33.528, 35.7632, 40.2336]
  x_vel = to_mph(x_vel)
  y_dist = [1.385, 1.394, 1.406, 1.421, 1.444, 1.474, 1.516, 1.534, 1.546, 1.568, 1.579, 1.593, 1.614]
  TR_relaxed = interp(v_ego, x_vel, y_dist)
  relaxed_mod_pos = [1.0, 1.0, 1.0, 1.0]
  relaxed_mod_neg = [1.0, 1.0, 1.0, 1.0]
  relaxed_mod_pos = interp(v_ego, p_mod_x, relaxed_mod_pos)
  relaxed_mod_neg = interp(v_ego, p_mod_x, relaxed_mod_neg)


  x_rel = [-20.0288, -15.6871, -11.1965, -7.8645, -4.9472, -3.0541, -2.2244, -1.5045, -0.7908, -0.3196, 0.0, 0.5588, 1.3682, 1.898, 2.7316, 4.4704]  # relative velocity values
  x_rel = to_mph(x_rel)
  y_rel = [0.62323, 0.49488, 0.40656, 0.32227, 0.23914, 0.12269, 0.10483, 0.08074, 0.04886, 0.0072, 0.0, -0.05648, -0.0792, -0.15675, -0.23289, -0.315]  # modification values

  TR_mod_pos = interp(-10, x_rel, y_rel)
  TR_mod_neg = interp(3.6, x_rel, y_rel)
  print('v_ego: {}'.format(v_ego))
  print('traffic: {}'.format(TR_traffic))
  pos_traffic = TR_traffic + TR_mod_pos * traffic_mod_pos
  neg_traffic = TR_traffic + TR_mod_neg * traffic_mod_neg
  print('pos: {}, neg: {}'.format(pos_traffic, neg_traffic))
  print()
  print('relaxed: {}'.format(TR_relaxed))
  pos_relaxed = TR_relaxed + TR_mod_pos * relaxed_mod_pos
  neg_relaxed = TR_relaxed + TR_mod_neg * relaxed_mod_neg
  print('pos: {}, neg: {}'.format(pos_relaxed, neg_relaxed))
  print('pos difference: {}%'.format(100*(1 - (pos_traffic / pos_relaxed))))
  print('neg difference: {}%'.format(100*(1 - (neg_traffic / neg_relaxed))))
  print('------')
