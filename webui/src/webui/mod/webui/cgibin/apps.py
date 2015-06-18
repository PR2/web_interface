#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import os, sys, string, time, getopt, re

import neo_cgi, neo_util, neo_cs

from pyclearsilver import CSPage
from pyclearsilver import odb
from pyclearsilver.log import *

import MBPage
import db_webui
import webutil
import config

import launchman.app


class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    self.db = db_webui.initSchema()

  def display(self, hdf):
    webutil.grabTopics(hdf, [])

    hdf.setValue("CGI.now", str(time.time()))
    webutil.set_tabs(hdf, ["apps"])

    apps = self.db.apps.fetchAllRows()
    prefix = "CGI.cur.apps"
    i = 0
    for app in apps:
      i = i + 1
      aprefix = prefix + ".%d" % i
      app.hdfExport(aprefix, hdf)
      app.fetchApp(aprefix, hdf)
      
    # experimental new stuff
    apps = webutil.list_apps()
    categories = {}
    user_record = self.authdb.users.lookup(self.username)    
    user_apps = user_record.favorite_apps_list()
    
    taskids = [db_webui.path2taskid(app) for app in apps]    
    actual_apps = []
    
    for user_app in user_apps:
        if user_app in taskids:
            actual_apps.append(user_app)
    categories["Favorites"] = actual_apps

    n = 0
    for appfn in apps:
      taskid = db_webui.path2taskid(appfn)
      n = n + 1
      prefix = "CGI.cur.available_apps.%s" % taskid
      _app = launchman.app.App(taskid)
      doc = _app.load_yaml()

      iapp = self.db.apps.lookup(taskid=taskid)
      if iapp:
        pass

      # only show apps relevant to this type of robot
      robot_type = config.get_robot_type()
      if "robot" in doc and robot_type != doc["robot"]:
          continue

      try:
        category_apps = categories[doc.get("category", "Other")]
      except KeyError:
        category_apps = []
        categories[doc.get("category", "Other")] = category_apps
      category_apps.append(taskid)
      
      hdf.setValue(prefix + ".taskid", taskid)
      for key, val in doc.items():
        if type(val) is list:
          for i in range(0,len(val)):
            hdf.setValue(prefix + "." + key + "." + str(i), val[i])
        elif val is not None:
          hdf.setValue(prefix + "." + key, val)
        else:
          hdf.setValue(prefix + "." + key, '')

      if user_record.is_favorite_app(taskid):
        hdf.setValue(prefix + ".favorite", "1")
    
    def compare_categories(a, b):
      if a[0] == "Favorites":
        return -1
      if b[0] == "Favorites":
        return 1
      else:
        return cmp(a, b)
      
    prefix = "CGI.cur.categories"
    i = 0
    for category, apps in sorted(categories.items(), compare_categories):
      hdf.setValue(prefix + ".%d" % i, category)
      j = 0
      for app in apps:
        hdf.setValue(prefix + ".%d.apps.%d" % (i, j), app)
        j = j + 1
      i = i + 1
      
def run(context):
  return MyPage(context, pagename="apps", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
