#!/usr/bin/env python

import nstart
import config
import os, sys, string, time

from pyclearsilver.log import *

from pyclearsilver.CSPage import Context
import neo_cgi, neo_cs, neo_util
from MBPage import MBPage

from auth import browserauth
from auth import cookieauth
from auth import db_auth
from auth import pwauth

from pyclearsilver import wordwrap
from email import MIMEText, Generator, Parser
from cStringIO import StringIO

from web_msgs.msg import WebEvent
import rospy

class SignInPage(MBPage):
    def setup(self, hdf):
      self.requestURI = hdf.getValue("Query.request", "")
      self.authdb = db_auth.initSchema()

    def display0(self, hdf):
        q_signout = hdf.getIntValue("Query.signout",0)
        self.requestURI = hdf.getValue("Query.request", "")
        if self.requestURI:
          hdf.setValue("CGI.cur.request", self.requestURI)

        if q_signout:
          cookieauth.clearLoginCookie(self.ncgi, self.username)

    def display(self, hdf):
      self.redirectUri(self.default_app_path())

    def requestChangePassword(self):
      hdf = self.ncgi.hdf
      requestURI = hdf.getValue("CGI.RequestURI", "")
      rurl = config.gBaseURL + "login/changePassword.py"
      self.redirectUri(rurl + "?q=1&request=" + neo_cgi.urlEscape(config.gBaseURL + "webui/"))
      
    def Action_Logout(self, hdf):
      warn("action logout called")

      self.ncgi.cookieClear("inactive")

      if self.is_active_user():
          warn("removing active user")
          self.remove_active_user()
      else:
          warn("logging out non-active user")

      # publish a web event that we logged out
      pub = rospy.Publisher("/webui/events", WebEvent)
      rospy.init_node("webui_login", anonymous=True)
      msg = WebEvent()
      msg.source = "user"
      msg.type = "logout"
      msg.data = self.username
      pub.publish(msg)

      if config.get_robot_type().startswith("texas"):
          self.redirectUri(config.gLobbyReturnPage + "?robot_name=" + hdf.getValue('CGI.Robot', ""))
      else:
          # don't clear login cookie for texai since the lobby will handle it
          domain = hdf.getValue("HTTP.Host", "")
          cookieauth.clearLoginCookie(self.ncgi, self.username, domain)
          self.redirectUri(self.default_app_path())

    def Action_Login(self, hdf):

        q_username =  hdf.getValue("Query.username","")
        q_passwordHash = hdf.getValue("Query.password","")
        q_persist = hdf.getValue("Query.persist","0")

        try: q_persist = int(q_persist)
        except ValueError: q_persist = 0

        default_requestURI = config.gBaseURL + "%s/" % config.gDefaultModule

        warn("requestURI", self.requestURI)

        if not self.requestURI:
          self.requestURI = default_requestURI


        wwwhostname = hdf.getValue("HTTP.Host", "")

        rurl = self.http + wwwhostname + config.gBaseURL + "login/signin0.py"

        warn("signin.py", rurl)

        # open login db to get pw
        userRec = self.authdb.users.lookup(q_username)
        
        if not userRec:
          warn("signin.py", "login failure (%s) unknown user" % q_username)
          self.redirectUri(rurl + "?err=Invalid+Login&request=%s" % neo_cgi.urlEscape(self.requestURI))

        q_password = pwauth.unmungePassword(q_passwordHash)

        ipaddr = hdf.getValue("CGI.RemoteAddress", "Unknown")
        browserid = browserauth.getBrowserCookie(self.ncgi)

        now = time.time()

        

        loginRow = self.authdb.login.newRow()
        loginRow.uid = userRec.uid
        loginRow.username = userRec.username
        loginRow.ipaddr = ipaddr
        loginRow.browserid = browserid

        if userRec.checkPassword(q_password) == 0:
          warn("signin.py", "login failure (%s) password mismatch" % q_username, q_password)
          loginRow.loginType = 0
          loginRow.save()


          url = rurl + "?err=Invalid+Login&request=%s" % neo_cgi.urlEscape(self.requestURI)
          warn("redirecting to", url)
          self.redirectUri(url)
          return

        # ----------- success!!! ------------------
        # generate cookie

        loginRow.loginType = 1
        loginRow.save()

        cookieauth.issueLoginCookie(self.ncgi, self.authdb, q_username, userRec.pw_hash, q_persist)

        if userRec.changePassword == 1:
          self.requestChangePassword()
          return

        # publish a web event that we logged in
        pub = rospy.Publisher("/webui/events", WebEvent)
        rospy.init_node("webui_login", anonymous=True)
        msg = WebEvent()
        msg.source = "user"
        msg.type = "login (local)"
        msg.data = self.username
        pub.publish(msg)

        # redirect to the main page
        self.redirectUri(self.requestURI)



    def __del__(self):
        if self.authdb:
            self.authdb.close()
            self.authdb = None

def run(context):
    page = SignInPage(context, pagename="signin",nologin=1)
    return page

def main(context):
  page = run(context)
  page.start()
  

if __name__ == "__main__":
    main(Context())
