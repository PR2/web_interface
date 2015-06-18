# this starts up the python enviroment
# 
# The root dir should point to the top of the python tree

import os, sys

ROOT_DIR = "../../.."

sys.path.insert(0,ROOT_DIR)
sys.path.insert(0,"../..")

from neo_paths import paths
sys.path = paths(ROOT_DIR) + sys.path

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
