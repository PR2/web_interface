#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import os, sys, string, time, getopt, re
from pyclearsilver.log import *

import neo_cgi, neo_util, neo_cs

from pyclearsilver import CSPage
from pyclearsilver import odb
import launchman.app

import MBPage
import db_webui
import webutil

class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    self.db = db_webui.initSchema()
    self.appgroup = self.db.appgroups.lookup(id=hdf.getValue("Query.id", ""))
    self.appids = self.appgroup.appIdList()
 
  def display(self, hdf):
    webutil.set_tabs(hdf, ["apps", "appgroup"])
   
    i = 0
    for taskid in self.appids:
      doc = launchman.app.App(taskid).load_yaml()
  
      prefix = "CGI.cur.app_group.apps.%s" % i
      hdf.setValue(prefix + "." + "taskid", taskid)
      for key, val in doc.items():
        if val is not None:
          hdf.setValue(prefix + "." + key, val)
        else:
          hdf.setValue(prefix + "." + key, '')
      i += 1

    self.appgroup.hdfExport("CGI.cur.app_group", hdf)

    # experimental new stuff
    apps = webutil.list_apps()
    n = 0
    for appfn in apps:
      taskid = db_webui.path2taskid(appfn)
      prefix = "CGI.cur.available_apps.%d" % n
      _app = launchman.app.App(taskid)
      doc = _app.load_yaml()

      if taskid in self.appids:
        pass
      else: 
        n = n + 1 
        hdf.setValue(prefix + "." + "taskid", taskid)
        for key, val in doc.items():
          if val is not None:
            hdf.setValue(prefix + "." + key, val)
          else:
            hdf.setValue(prefix + "." + key, '')

  def Action_AddApp(self, hdf):
    taskid = hdf.getValue("Query.taskid", "")
    
    if taskid not in self.appids:
      user_id = self.authdb.users.lookup(hdf.getValue("CGI.Login", "")).uid

      self.appids.append(taskid)
      self.appgroup.appids = string.join(self.appids, ',')
      self.appgroup.save()

      
def run(context):
  return MyPage(context, pagename="appgroup", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
