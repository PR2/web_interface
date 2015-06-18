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
from auth import db_auth

import roslib
import roslib.scriptutil
#import rospy
import webutil

class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    self.db = db_webui.initSchema()
    self.auth_db = db_auth.initSchema()
    
  def display(self, hdf):
    webutil.set_tabs(hdf, ["admin", "users"])
    #users = self.auth_db.getAllUsers()
    user_records = self.auth_db.users.fetchAllRows()
    user_records.hdfExport("CGI.cur.users", hdf)
    
  def Action_DeleteUser(self, hdf):
    username = hdf.getValue("Query.username", "").strip()
    row = self.authdb.users.lookup(username=username)
    row.delete()
    self.redirectUri("users.py")

def run(context):
  return MyPage(context, pagename="users", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
