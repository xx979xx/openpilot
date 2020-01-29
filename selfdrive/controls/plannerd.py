#!/usr/bin/env python3
import gc

from cereal import car
from common.params import Params
from common.realtime import set_realtime_priority
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.planner import Planner
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.pathplanner import PathPlanner
import cereal.messaging as messaging
import cereal.messaging_arne as messaging_arne

def plannerd_thread(sm=None, pm=None, arne_sm=None):
  gc.disable()

  # start the loop
  set_realtime_priority(2)

  cloudlog.info("plannerd is waiting for CarParams")
  CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  PL = Planner(CP)
  PP = PathPlanner(CP)

  VM = VehicleModel(CP)

  if sm is None:
    sm = messaging.SubMaster(['carState', 'controlsState', 'radarState', 'model', 'liveParameters', 'liveMapData'])
  if arne_sm is None:
    arne_sm = messaging_arne.SubMaster(['arne182Status']) # , 'latControl'])
  if pm is None:
    pm = messaging.PubMaster(['plan', 'liveLongitudinalMpc', 'pathPlan', 'liveMpc'])

  sm['liveParameters'].valid = True
  sm['liveParameters'].sensorValid = True
  sm['liveParameters'].steerRatio = CP.steerRatio
  sm['liveParameters'].stiffnessFactor = 1.0

  while True:
    sm.update()
    arne_sm.update(0)
    
    if sm.updated['model']:
      PP.update(sm, pm, CP, VM)
    if sm.updated['radarState']:
      PL.update(sm, pm, CP, VM, PP, arne_sm)


def main(sm=None, pm=None):
  plannerd_thread(sm, pm)


if __name__ == "__main__":
  main()
