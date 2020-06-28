from cereal import car
import numpy as np

actuators = car.CarControl.Actuators.new_message()

actuators.gas = float(np.float64(0.5362536253890579305093859038673548953625362538905793050938590386735489))
print('success!')
