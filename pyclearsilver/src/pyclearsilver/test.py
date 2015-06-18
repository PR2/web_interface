#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""


import os, sys, string, time, getopt
from log import *

import odb
import odb_sqlite3


def test(name, email):
  print dir()
  


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

  test("scott", "hassan@dotfunk.com")


if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
