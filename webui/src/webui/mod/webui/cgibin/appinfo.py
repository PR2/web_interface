#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import os, sys, string, time, getopt, re
from pyclearsilver.log import *

import neo_cgi, neo_util, neo_cs

from pyclearsilver import CSPage
from pyclearsilver import odb
from launchman import app

import MBPage
import db_webui
import webutil

class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    self.db = db_webui.initSchema()

  def display(self, hdf):
    webutil.set_tabs(hdf, ["apps", "appinfo"])
    webutil.grabTopics(hdf, [])
    taskid = hdf.getValue("Query.taskid", "")
    doc = app.App(taskid).load_yaml()
  
    prefix = "CGI.cur.app"
    hdf.setValue(prefix + "." + "taskid", taskid)
    for key, val in doc.items():
      if type(val) is list:
        for i in range(0,len(val)):
          hdf.setValue(prefix + "." + key + "." + str(i), val[i])
      elif val is not None:
        hdf.setValue(prefix + "." + key, val)
      else:
        hdf.setValue(prefix + "." + key, '')

    # for favorite apps
    user_record = self.authdb.users.lookup(self.username)
    hdf.setValue("CGI.cur.user.favorite_apps", user_record.favorite_apps)

    if user_record.is_favorite_app(taskid):
      hdf.setValue("CGI.cur.app.favorite", "1")

  def Action_Favorites(self, hdf):
    taskid = hdf.getValue("Query.taskid", "")
    doc = app.App(taskid).load_yaml()
  
    prefix = "CGI.cur.app"
    hdf.setValue(prefix + "." + "taskid", taskid)

    # for favorite apps
    user_record = self.authdb.users.lookup(self.username)
    if hdf.getValue("Query.set_favorite", "") == "1":
      user_record.add_favorite_app(taskid)
    elif hdf.getValue("Query.set_favorite", "") == "0":
      user_record.remove_favorite_app(taskid)
      
    if user_record.is_favorite_app(taskid):
      hdf.setValue("CGI.cur.favorite", "1")
    
    hdf.setValue("Content", "appinfo_favorites.cs")
      
def run(context):
  return MyPage(context, pagename="appinfo", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
