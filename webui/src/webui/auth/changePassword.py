#! /usr/bin/env python

"""
usage: %(progname)s username
"""


import nstart
import os, sys, string, time, getopt

from pyclearsilver.log import *

import config

from pyclearsilver import odb, hdfhelp, odb_sqlite3
from pyclearsilver import CSPage

from pyclearsilver.odb import *
import db_auth



def changePassword(db, username, password):
  row = db.users.lookup(username)
  row.setPassword(password)
  row.changePassword = 1
  row.save()


def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug"])

  testflag = 0
  if len(args) != 1:
    usage(progname)
    return

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

  db = db_auth.initSchema(create=0)

  username = args[0]
  import getpass
  password1 = getpass.getpass("Password 1:")
  password2 = getpass.getpass("Password 2:")
  if password1 != password2:
    print "password mismatch."
    return

  changePassword(db, username, password1)
    



if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)

  
  
