#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import nstart

import os, sys, string, time, getopt, re
from pyclearsilver.log import *

import neo_cgi, neo_util, neo_cs

from pyclearsilver import CSPage
from pyclearsilver import odb

import MBPage
import db_webui

import roslib
import roslib.scriptutil
#import rospy
import webutil

class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    self.db = db_webui.initSchema()
    
  def display(self, hdf):
    webutil.set_tabs(hdf, ["admin", "users"])
    username = hdf.getValue("Query.username", "").strip()
    row = self.authdb.users.lookup(username=username)
    row.hdfExport("CGI.cur.user", hdf)

  def Action_SaveUser(self, hdf):
    # load params from request
    username = hdf.getValue("Query.username", "").strip()
    role = hdf.getValue("Query.role", "").strip()
      
    # save the new user record
    try:
      row = self.authdb.users.lookup(username=username)
      row.role = hdf.getValue("Query.role", "")
      row.save()
    except:
      hdf.setValue("CGI.cur.error_message.password", "There was an error when trying to save this user.")
    
    self.redirectUri("users.py")
    
def run(context):
  return MyPage(context, pagename="edituser", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
