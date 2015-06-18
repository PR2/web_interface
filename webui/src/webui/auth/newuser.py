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

def createuser(db, username, password, role=""):
  row = db.users.new(username, password)
  row.role = role
  row.changePassword = 1
  row.save()


def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug", "admin"])

  testflag = 0
  if len(args) not in (1, 2):
    usage(progname)
    return

  role = ""

  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1
    elif field == "--admin":
      role = "admin"

  if testflag:
    test()
    return

  db = db_auth.initSchema(create=0)

  username = args[0]
  
  if len(args) > 1:
    password1 = args[1]
    print "---------------"
    print "Creating new user:"
    print "  Username: %s" % username
    print "  Password: %s" % password1
    print "You will be asked to change this password the first time you log in to the web interface."
  else:
    import getpass

    print "New User: %s" % username
    print "---------------------------"
    print 
    while 1:
      print "Please enter a password:"
      password1 = getpass.getpass("Password 1:")
      password2 = getpass.getpass("Password 2:")
      if password1 != password2:
        print "password mismatch."
        continue
      else:
        break

  user = createuser(db, username, password1, role=role)
    



if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)

  
  
