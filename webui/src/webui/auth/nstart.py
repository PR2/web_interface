# this starts up the python enviroment
# 
# The root dir should point to the top of the python tree

import os, sys

PKG = 'webui' # this package name
import roslib; roslib.load_manifest(PKG) 

if 1:
  script_dir = ''
  try:
    script_name = sys.argv[0]
    while 1:
      script_dir = os.path.dirname(script_name)
      if not os.path.islink(script_name):
        break
      script_name = os.path.join(script_dir, os.readlink(script_name))
  except KeyError, reason:
    pass
  except AttributeError, reason:
    pass

  script_dir = os.path.join(os.getcwd(), script_dir)
  script_dir = os.path.normpath(script_dir)

  path = script_dir

  ROOT_DIR = os.path.join(path, "..")
  ROOT_DIR = os.path.normpath(ROOT_DIR)
else:
  ROOT_DIR = ".."

sys.path.append(ROOT_DIR)

from neo_paths import paths
sys.path = paths(ROOT_DIR) + sys.path
sys.path.append(os.path.join(ROOT_DIR, "mod/mail"))

# don't put anything above this because the path isn't
# extended yet...

import neo_cgi
try:
  # newer versions have an update function that will guaruntee that
  # neo_util and neo_cs are also loaded when used with non single interpreter
  # versions of PyApache
  neo_cgi.update()
except:
  pass
