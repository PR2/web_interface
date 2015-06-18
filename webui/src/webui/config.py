
import os
import yaml

# config data
gRequireUsername = 0

gWebUser = "apache"
gWebUserID = None  # apache
gWebGroupID = None # apache

gBaseURL = "/webui/"
gROSURL = "/ros"

gAuthVCode = 1574444059
gAuthSalt = "ir"

gWebUserID = 33  # apache
gWebGroupID = 33 # apache

gDomain = "willowgarage.com"
gROSBridgePort = ":8068"

LOGIN_TIMEOUT = 60*60*4

# 1 hour
REFRESH_COOKIE_TIMEOUT = 60*60

ROS_VAR_DIR = os.environ.get("ROS_VAR_DIR", "/var/ros")
if ROS_VAR_DIR == "":
  ROS_VAR_DIR = "/var/ros"

ACTIVE_USER_FILE = os.path.join(ROS_VAR_DIR, "active_user.dat")
ACTIVE_USER_TIMEOUT = 3600 # after one hour of inactivity, the active user will stop being active

VALID_USER_COOKIE_FILE = os.path.join(ROS_VAR_DIR, "user_cookie.dat")

gDBPath = os.path.join(ROS_VAR_DIR, "db")

ROS_CONFIG_DIR = "/etc/ros/env"
ROS_CONFIG_FILE = "/etc/ros/robot.yaml"

def getSiteDBPath(module):
  path = os.path.join(gDBPath, module)
  return path
  
def getDBPath(module):
  path = os.path.join(gDBPath, module)
  return path
  
def createDBPath(path):
  if not os.path.isdir(path):
    os.makedirs(path, 0700)
    #webChown(path)
  
def webChown(path):
  if gWebUserID is not None and gWebGroupID is not None:
    os.chown(path, gWebUserID, gWebGroupID)
  
def get_robot_type():
  if os.path.exists(ROS_CONFIG_FILE):
    try:
      with open(ROS_CONFIG_FILE, 'r') as conf:
        return yaml.load(conf)['robot']['type']
    except:
      pass
  try:
    return open(os.path.join(ROS_CONFIG_DIR, "ROBOT")).read().strip()
  except IOError:
    return "desktop"
  
def get_robot_name():
  if os.path.exists(ROS_CONFIG_FILE):
    try:
      with open(ROS_CONFIG_FILE, 'r') as conf:
        return yaml.load(conf)['robot']['name']
    except:
      pass
  try:
    return open(os.path.join(ROS_CONFIG_DIR, "ROBOT_NAME")).read().strip()
  except IOError:
    return "localhost"
  
if get_robot_type().startswith("texas"):
    gDefaultModule = "app/texas_web_teleop/texas_web_teleop"
else:
    gDefaultModule = "webui"
    
gDefaultPage = "webui"

gLobby = None
gHomeServer = None

if get_robot_type().startswith("texas"):
    gLobby = "http://priv1.texai.com"
    gLobbyHost = "priv1.texai.com"
    gHomeServer = gLobby + "/lobby/lobby/robot_data.py"
    gDomain = "texai.com"
    gROSBridgePort = ""
    gLobbyReturnPage = "https://www.texai.com/lobby/lobby/disconnect.py"
