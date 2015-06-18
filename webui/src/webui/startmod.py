#! /usr/bin/env python

import os, sys

from mod_python import apache

env = {}

def handler(req):
  env = apache.build_cgi_env(req)

  if os.path.exists("/etc/ros/env"):
    keys = os.listdir("/etc/ros/env/")
    for key in keys:
      if key not in os.environ:
        os.environ[key] = open("/etc/ros/env/%s" % key).read().strip()

  ros_root = os.environ['ROS_ROOT']

  roslib_src = os.path.join(ros_root, "core/roslib/src")
  if not roslib_src in sys.path:  
    sys.path.append(roslib_src)

  if os.environ['PATH'].find(ros_root) == -1:
    os.environ['PATH'] = os.path.join(ros_root, 'bin') + ":" + os.environ.get('PATH', '')

  os.environ['HOME'] = '/tmp'
  os.environ['ROS_CACHE_TIMEOUT'] = '3600'

  if env.get('PYTHON_RELOADER', '').lower() == 'on':
    from pyclearsilver import autoreloader

  import cgistarter
  import config

  #sys.stderr.write("path:" + os.getcwd())
  path,f = os.path.split(__file__)
  #sys.stderr.write("path: %s\n"  % path)
  cwd = path
  #os.chdir(path)

  cgistarter.setConfig(config)
  return cgistarter.handler(req, cwd)

