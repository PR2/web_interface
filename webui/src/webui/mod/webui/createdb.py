#! /usr/bin/env python

"""
usage: %(progname)s
"""


import os, sys, string, time, getopt

PKG = 'webui' # this package name
import roslib; roslib.load_manifest(PKG) 

from pyclearsilver.log import *

import db_webui

def test():
  pass

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug"])

  testflag = 0

  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1

  if testflag:
    test()
    return

  db = db_webui.initSchema(create=1)

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
