import ftplib
import json
import string
import random
import os
from common.params import Params
from common.op_params import opParams
op_params = opParams()
uniqueID = op_params.get('uniqueID', None)

def upload_data():
  filepath = "/data/openpilot/selfdrive/data_collection/gps-data"
  if os.path.isfile(filepath):
    if uniqueID is None:
      op_params.put('uniqueID', ''.join([random.choice(string.ascii_lowercase+string.ascii_uppercase+string.digits) for i in range(15)]))
    try:
      username = op_params.get('uniqueID', None)
      try:
        with open("/data/data/ai.comma.plus.offroad/files/persistStore/persist-auth", "r") as f:
          auth = json.loads(f.read())
        auth = json.loads(auth['commaUser'])
        if auth and str(auth['username']) != "":
          username = str(auth['username'])
      except:
        pass

      params = Params()
      car = params.get('CachedFingerprint')
      if car is not None:
        car = json.loads(car)
        username+="-{}".format(car[0])

      filename = "gps-data.{}".format(random.randint(1,99999))

      ftp = ftplib.FTP("arneschwarck.dyndns.org")
      ftp.login("openpilot", "communitypilot")
      with open(filepath, "rb") as f:
        try:
          ftp.mkd("/{}".format(username))
        except:
          pass
        ftp.storbinary("STOR /{}/{}".format(username, filename), f)
      ftp.quit()
      os.remove(filepath)
      return True
    except:
      return False
  else:
    return False
