#!/usr/bin/env python

import nstart
import config
import os, sys, string, time

from pyclearsilver.log import *

from pyclearsilver.CSPage import Context, CSPage
import neo_cgi

from auth import cookieauth
from auth import pwauth

import xss

import MBPage

class SignInPage(MBPage.MBPage):
    def setup(self):
      hdf = self.ncgi.hdf
      self.requestURI = hdf.getValue("Query.request", "")

    def display(self):
        hdf = self.ncgi.hdf
        q_signout = hdf.getIntValue("Query.signout",0)
        self.requestURI = hdf.getValue("Query.request", "")

        q_username = xss.xssescape(string.lower(hdf.getValue("Query.username","")))

        hdf.setValue("CGI.username", q_username)

        if self.requestURI:
          hdf.setValue("CGI.cur.request", self.requestURI)

        if q_signout:
          cookieauth.clearLoginCookie(self.ncgi, self.username)

    def Action_Login(self):
        hdf = self.ncgi.hdf

        q_username = xss.xssescape(string.lower(hdf.getValue("Query.username","")))
        q_password = hdf.getValue("Query.password","")
        q_persist = hdf.getValue("Query.persist","0")

        q_password_Hash = pwauth.mungePassword(q_password)

        if not self.requestURI:
            self.requestURI = config.gBaseURL + "%s/" % config.gDefaultModule

        hostname = hdf.getValue("HTTP.Host", "")

        # open login db to get pw
        newhost = hostname

        cookieauth.setPersistCookie(self.ncgi, q_persist)

        url = self.http + newhost + config.gBaseURL + "login/signin.py?password=%s&persist=%s&Action.Login=1&request=%s&username=%s" % (neo_cgi.urlEscape(q_password_Hash), q_persist, neo_cgi.urlEscape(self.requestURI), q_username)
        warn("signin0.py", "redirecting to url", url)
        self.redirectUri(url)

def run(context):
    page = SignInPage(context, pagename="signin0", nologin=1)
    return page

def main(context):
  page = run(context)
  page.start()
  

if __name__ == "__main__":
    main(Context())
