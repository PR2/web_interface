#! /usr/bin/env python

import os, sys, string, time, getopt
import urllib
import roslib

import config

def grabTopics(hdf, topics):
  topics = topics + ["/power_state", "/power_board_state", "/app_status"]
  topicList = []
  for topic in topics: topicList.append("topic=%s" % topic)
  topics = string.join(topicList, "&")

  url = "http://localhost:%s/ros/receive?since=0&nowait=1&%s" % (config.gROSBridgePort, topics)
  try:
    fp = urllib.urlopen(url)
    body = fp.read()
    fp.close()

    hdf.setValue("CGI.cur.messages", body)
  except:
    pass

def list_apps():
  import subprocess
  import glob

  apps = []
  cmd = ["rospack", "depends-on", "webui"]
  p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
  while 1:
    line = p.stdout.readline()
    if not line: break
    pkg = line.strip()

    path = roslib.packages.get_pkg_dir(pkg)
    files = glob.glob(os.path.join(path, "*.app"))
    for app in files:
      apps.append(app)
  return apps

def set_tabs(hdf, tabs):
  i = 0
  for tab in tabs:
    hdf.setValue("CGI.cur.tabs.%d" % i, tab)
    i += 1
    
def hdf_array(hdf, array, prefix):
  i = 0
  for element in array:
    hdf.setValue("%s.%s" % (prefix, i), element)
    i += 1

