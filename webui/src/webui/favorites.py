#! /usr/bin/env python

"""
usage: %(progname)s username 
    add [taskids...]
    list
    del [taskids...]
"""

import roslib; roslib.load_manifest('webui')

import os, sys, string, time, getopt
import urllib

from auth import db_auth
import config

def test():
  pass

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help"])

  testflag = 0
  if len(args) < 2:
    usage(progname)
    return
  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
  db = db_auth.initSchema()

  username = args[0]
  cmd = args[1]


  user = db.users.lookup(username=username)
  if cmd == "add":
    for taskid in args[1:]:
      user.add_favorite_app(taskid)
  elif cmd == "list":
    for app in user.favorite_apps_list():
      print app
  elif cmd == "del":
    for taskid in args[1:]:
      user.remove_favorite_app(taskid)
    
    

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
