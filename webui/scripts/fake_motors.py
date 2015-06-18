#!/usr/bin/env python

PKG = 'webui' # this package name
NAME = 'fake_motors'

import roslib; roslib.load_manifest(PKG)

import sys
import time

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty,EmptyResponse

from optparse import OptionParser
def talker(options):
  pub = rospy.Publisher("pr2_etherCAT/motors_halted", Bool)
  rospy.init_node(NAME, anonymous=True)
  while not rospy.is_shutdown():
    out = Bool()
    out.data = options.halt
    pub.publish(out)
    rospy.sleep(1.0)

def halt_motors(req):
  options.halt = True
  return EmptyResponse()

def reset_motors(req):
  options.halt = False
  return EmptyResponse()

if __name__ == '__main__':
  parser = OptionParser()
  parser.add_option('--halt', action="store_true", dest="halt", default=True)
  (options, args) = parser.parse_args()
  h = rospy.Service('pr2_etherCAT/halt_motors', Empty, halt_motors)
  r = rospy.Service('pr2_etherCAT/reset_motors', Empty, reset_motors)
  try:
    talker(options)
  except KeyboardInterrupt, e:
    pass
  print "exiting"

