#!/usr/bin/env python 

import nstart
import config
import os, sys, string, time

from pyclearsilver.CSPage import Context
from MBPage import MBPage

import neo_cgi

from pyclearsilver.log import *

from auth import db_auth
from auth import cookieauth

class IndexPage(MBPage):
  def setup(self):
    hdf = self.ncgi.hdf
    self.requestURI = hdf.getValue("Query.request", "")
    if self.requestURI:
      hdf.setValue("CGI.cur.request", self.requestURI)

  def display(self):
    hdf = self.ncgi.hdf

    self.requestURI = hdf.getValue("Query.request", "")
    if not self.requestURI:
      self.requestURI = hdf.getValue("HTTP.Referer", "")

    if self.requestURI:
      hdf.setValue("CGI.cur.request", self.requestURI)


  def error(self, msg):
    self.redirectUri("changePassword.py?err=%s&request=%s" % (neo_cgi.urlEscape(msg), neo_cgi.urlEscape(self.requestURI)))

  def Action_changePassword(self):
    hdf = self.ncgi.hdf

    q_pw0 = hdf.getValue("Query.pw0","")

    q_pw1 = hdf.getValue("Query.pw1","")
    q_pw2 = hdf.getValue("Query.pw2","")


    requestURI = hdf.getValue("Query.request", "%swebui" % config.gBaseURL)

    if not requestURI:
      requestURI = config.gBaseURL + self.login + "/mail/prefs.py"

    if not self.login:
      self.redirectUri(self.requestURI)

    if not q_pw0:
      self.error("Old password is not complete.")

    if not q_pw1 or (q_pw1 != q_pw2):
      self.error("New passwords do not match.")

    authdb = db_auth.initSchema()
    userRec = authdb.users.lookup(self.login)

    if not userRec.checkPassword(q_pw0):
      self.error("Old password is invalid.")

    userRec.setPassword(q_pw1)

    cookieauth.issueLoginCookie(self.ncgi, authdb, self.login, userRec.pw_hash)

#    self.issueLoginCookie(self.login, self.MB.getOption("pw_hash"))

    warn("redirecting to", repr(self.requestURI))
    self.redirectUri(self.requestURI)
        

def run(context):
  return IndexPage(context, pagename="changePassword", checkActive=False)

def main(context):
  run(context).start()

if __name__ == "__main__":
    main(Context())
