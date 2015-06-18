#! /usr/bin/env python

"""
usage: %(progname)s robot robot_type web_user
"""

import os, sys, string, time, getopt
import pwd
import subprocess

def make_file(fn, default):
  if os.path.exists(fn): return

  open(fn, "w").write(default)

def install(robot, robot_type, web_user):
  config_dir = "/etc/ros/"
  var_dir = "/var/ros/"
  log_dir = "/var/log/ros/"

  user = pwd.getpwnam(web_user)
  for path in (config_dir, var_dir, log_dir):
    if not os.path.exists(path):
      os.mkdir(path, 0755)
    os.chown(path, user.pw_uid, user.pw_gid)

  make_file(os.path.join(config_dir, "user"), web_user)
  make_file(os.path.join(config_dir, "master"), "http://localhost:11311/")
  make_file(os.path.join(config_dir, "robot"), robot)
  make_file(os.path.join(config_dir, "robot_type"), robot_type)

  subprocess.call(["install", "setup.bash", os.path.join(config_dir, "setup.bash")])
  subprocess.call(["install", "apache.cfg", os.path.join(config_dir, "ros_webui_apache.cfg")])

  subprocess.call(["install", "webui.init", "/usr/bin/webui"])

def test():
  pass

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug"])

  testflag = 0

  if len(args) == 0:
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

  robot = args[0]
  robot_type = args[1]
  web_user = args[2]

  install(robot, robot_type, web_user)


if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
