#! /usr/bin/env python

import subprocess
import threading
import time
import sys, os, getopt

import roslib.scriptutil
from roslib.names import ns_join, get_ros_namespace, make_caller_id, make_global_ns, GLOBALNS

PKG_NAME = 'rosweb'
import roslib; roslib.load_manifest(PKG_NAME)

def _get_caller_id():
    return make_caller_id('rosparam-%s'%os.getpid())

class Params(roslib.message.Message):
  __slots__ = ('parameters', )
  def __init__(self, params):
    self.parameters = params

def succeed(args):
    code, msg, val = args
    if code != 1:
        raise RosTopicException("remote call failed: %s"%msg)
    return val

class ParamThread(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)

    self.callback = None

  def setCallback(self, callback):
    self.callback = callback

  def flatten(self, params, path="/"):
    l = []
    for k,v in params.items():
      _path=path + k
      if type(v) == type({}):
        l = l + self.flatten(v, _path + "/")
      else:
        l.append((_path, str(v)))
    return l

  def getParams(self):
    master = roslib.scriptutil.get_master()

    _c, _t, params = master.getParam(_get_caller_id(), "/")
    paramlist = self.flatten(params)
    paramlist.sort()

    return paramlist

  def run(self):
    lastval = ()
    while 1:
      try:
        val = self.getParams()
        if val != lastval:
          self.callback(Params(val))
          lastval = val
      except:
        pass
      time.sleep(3)

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug"])

  p = ParamThread()
  p.callback = None
  p.run()

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)

