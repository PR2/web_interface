#!/usr/bin/env python

import nstart
import config
import os, sys, string, time

from pyclearsilver.log import *
	
from pyclearsilver.CSPage import Context
import neo_cgi, neo_cs, neo_util
from MBPage import MBPage

class ActivePage(MBPage):
    def setup(self, hdf):
      self.requestURI = hdf.getValue("Query.request", "")
      if not self.requestURI:
        self.requestURI = self.default_app_path()

    def display(self, hdf):
      pass

    def __del__(self):
      if self.authdb:
        self.authdb.close()
        self.authdb = None

    def Action_Display(self, hdf):
      if hdf.getValue("Query.dismiss_active_warning", "") == "1":
        user_record = self.authdb.users.lookup(self.username)
        user_record.dismiss_notice("active_warning")
      self.display(hdf)

    def Action_MakeActive(self, hdf):
      # if one of the following are true, make the user active
      if (not self.get_active_user()) or self.is_active_user() or hdf.getValue("Query.override", "") == "1":
        self.make_active_user(hdf)
        self.ncgi.cookieClear("inactive")

        # if the dismiss checkbox was checked, store it in the database
        if hdf.getValue("Query.dismiss_active_warning", "") == "1":
          user_record = self.authdb.users.lookup(self.username)
          user_record.dismiss_notice("active_warning")

        self.redirectUri(self.requestURI)
      else:
        # if this user has already dismissed the active warning page, make them active
        user_record = self.authdb.users.lookup(self.username)
        if user_record.notice_dismissed("active_warning"):
          self.make_active_user(hdf)
          self.ncgi.cookieClear("inactive")
          self.redirectUri(self.requestURI)
        else:
          # redirect to page warning about active user
          url = self.http + hdf.getValue("HTTP.Host", "") + config.gBaseURL + "active/active_warning.py?request=%s" % (neo_cgi.urlEscape(hdf.getValue("Query.request", "")))
          self.redirectUri(url)

  
    def Action_MakeInactive(self, hdf):
      self.ncgi.cookieSet("inactive", "1")
      if self.is_active_user():
        self.remove_active_user()
      self.redirectUri(self.requestURI)

    def Action_DismissNotice(self, hdf):
        self.pagename = "notice"
        user_record = self.authdb.users.lookup(self.username)
        notice = hdf.getValue("Query.notice", "")
        if notice != "":
            user_record.dismiss_notice(notice)
            hdf.setValue("CGI.cur.value", "OK")

    def Action_NoticeDismissed(self, hdf):
        self.pagename = "notice"
        user_record = self.authdb.users.lookup(self.username)
        notice = hdf.getValue("Query.notice", "")
        if user_record.notice_dismissed(notice):
            hdf.setValue("CGI.cur.value", "YES")
        else:
            hdf.setValue("CGI.cur.value", "NO")
        
def run(context):
    page = ActivePage(context, pagename="active", nologin=False)
    return page

def main(context):
  page = run(context)
  page.start()
  

if __name__ == "__main__":
    main(Context())
