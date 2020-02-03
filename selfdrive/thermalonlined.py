#!/usr/bin/env python3.7
import psutil
from common.realtime import DT_TRML
import cereal.messaging_arne as messaging
from selfdrive.loggerd.config import get_available_percent

def read_tz(x, clip=True):
  try:
    with open("/sys/devices/virtual/thermal/thermal_zone%d/temp" % x) as f:
      ret = int(f.read())
      if clip:
        ret = max(0, ret)
  except FileNotFoundError:
    return 0

  return ret

def read_thermal():
  dat = messaging.new_message()
  dat.init('thermalonline')
  dat.thermalonline.cpu0 = read_tz(5)
  dat.thermalonline.cpu1 = read_tz(7)
  dat.thermalonline.cpu2 = read_tz(10)
  dat.thermalonline.cpu3 = read_tz(12)
  dat.thermalonline.mem = read_tz(2)
  dat.thermalonline.gpu = read_tz(16)
  dat.thermalonline.bat = read_tz(29)
  dat.thermalonline.pa0 = read_tz(25)
  return dat



def thermalonlined_thread():
  thermal_sock = messaging.pub_sock('thermalonline')

  count = 0

  while 1:
    # report to server once per seoncd
    if (count % int(1. / DT_TRML)) == 0:
      msg = read_thermal()

      msg.thermalonline.freeSpace = get_available_percent(default=100.0) / 100.0
      msg.thermalonline.memUsedPercent = int(round(psutil.virtual_memory().percent))
      msg.thermalonline.cpuPerc = int(round(psutil.cpu_percent()))

      try:
        with open("/sys/class/power_supply/battery/capacity") as f:
          msg.thermalonline.batteryPercent = int(f.read())
        with open("/sys/class/power_supply/battery/status") as f:
          msg.thermalonline.batteryStatus = f.read().strip()
        with open("/sys/class/power_supply/battery/current_now") as f:
          msg.thermalonline.batteryCurrent = int(f.read())
        with open("/sys/class/power_supply/battery/voltage_now") as f:
          msg.thermalonline.batteryVoltage = int(f.read())
        with open("/sys/class/power_supply/usb/present") as f:
          msg.thermalonline.usbOnline = bool(int(f.read()))
      except FileNotFoundError:
        pass

      thermal_sock.send(msg.to_bytes())
    count += 1


def main(gctx=None):
  thermalonlined_thread()

if __name__ == "__main__":
  main()
