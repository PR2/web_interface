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

  def Action_CreateUser(self, hdf):
    # load params from request
    username = hdf.getValue("Query.username", "").strip()
    password = hdf.getValue("Query.password", "").strip()
    role = hdf.getValue("Query.role", "").strip()
    
    # check some error conditions
    if self.authdb.users.lookup(username=username):
      hdf.setValue("CGI.cur.error_message.username", "User %s already exists." % username)
    if len(username) < 3:
      hdf.setValue("CGI.cur.error_message.username", "Please specify a user name (3 or more characters).")
    if len(password) < 3:
      hdf.setValue("CGI.cur.error_message.password", "Please specify a password (3 or more characters).")   
    if hdf.getObj("CGI.cur.error_message"):
      return
      
    # save the new user record
    try:
      row = self.authdb.users.newRow()
      row.username = hdf.getValue("Query.username", "")
      new_password = hdf.getValue("Query.password", "")
      row.setPassword(new_password)
      row.role = hdf.getValue("Query.role", "")
      row.save()
    except:
      hdf.setValue("CGI.cur.error_message.password", "There was an error when trying to save this user.")
    
    self.redirectUri("users.py")
    
def run(context):
  return MyPage(context, pagename="newuser", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
