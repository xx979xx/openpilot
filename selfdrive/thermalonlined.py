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
  dat.thermal.cpu0 = read_tz(5)
  dat.thermal.cpu1 = read_tz(7)
  dat.thermal.cpu2 = read_tz(10)
  dat.thermal.cpu3 = read_tz(12)
  dat.thermal.mem = read_tz(2)
  dat.thermal.gpu = read_tz(16)
  dat.thermal.bat = read_tz(29)
  dat.thermal.pa0 = read_tz(25)
  return dat



def thermalonlined_thread():
  thermal_sock = messaging.pub_sock('thermalonline')

  count = 0

  while 1:
    # report to server once per seoncd
    if (count % int(1. / DT_TRML)) == 0:
      msg = read_thermal()

      msg.thermal.freeSpace = get_available_percent(default=100.0) / 100.0
      msg.thermal.memUsedPercent = int(round(psutil.virtual_memory().percent))
      msg.thermal.cpuPerc = int(round(psutil.cpu_percent()))

      try:
        with open("/sys/class/power_supply/battery/capacity") as f:
          msg.thermal.batteryPercent = int(f.read())
        with open("/sys/class/power_supply/battery/status") as f:
          msg.thermal.batteryStatus = f.read().strip()
        with open("/sys/class/power_supply/battery/current_now") as f:
          msg.thermal.batteryCurrent = int(f.read())
        with open("/sys/class/power_supply/battery/voltage_now") as f:
          msg.thermal.batteryVoltage = int(f.read())
        with open("/sys/class/power_supply/usb/present") as f:
          msg.thermal.usbOnline = bool(int(f.read()))
      except FileNotFoundError:
        pass

      current_filter.update(msg.thermal.batteryCurrent / 1e6)

      # TODO: add car battery voltage check
      max_cpu_temp = max(msg.thermal.cpu0, msg.thermal.cpu1,
                         msg.thermal.cpu2, msg.thermal.cpu3) / 10.0
      max_comp_temp = max(max_cpu_temp, msg.thermal.mem / 10., msg.thermal.gpu / 10.)
      bat_temp = msg.thermal.bat/1000.
      thermal_sock.send(msg.to_bytes())
    count += 1


def main(gctx=None):
  thermalonlined_thread()

if __name__ == "__main__":
  main()
