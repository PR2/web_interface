#! /usr/bin/env python

import time
start = time.time()

import os,sys

if 0:
  ros_root = '/u/hassan/pr2/ros/'

  os.environ['ROS_ROOT'] = ros_root
  os.environ['ROS_PACKAGE_PATH'] = '/u/hassan/pr2/ros-pkg/'
  os.environ['ROS_MASTER_URI'] =  'http://localhost:11311/'
  rospythonpath = os.path.join(ros_root, 'core/roslib/src')
  os.environ['PYTHONPATH'] = rospythonpath
  os.environ['ROS_BOOST_PATH'] = '/opt/ros/'

  sys.path.insert(0, rospythonpath)

  os.environ['HOME'] = '/tmp'

os.environ['ROS_CACHE_TIMEOUT'] = '3600'
PKG = 'webui' # this package name
import roslib; roslib.load_manifest(PKG) 
ros_root = os.environ['ROS_ROOT']
os.environ['PATH'] = os.path.join(ros_root, "bin") + ":" + os.environ['PATH']

from pyclearsilver import log
from webui import cgistarter

import config

path,f = os.path.split(__file__)
os.chdir(path)

cgistarter.setConfig(config)

end = time.time()

try:
  start = time.time()
  cgistarter.main(sys.argv, sys.stdout, os.environ)
finally:
  end = time.time()
  #sys.stderr.write("serve time %s\n" % int((end-start)*1000))

