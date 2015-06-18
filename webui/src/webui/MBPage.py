
from pyclearsilver.CSPage import CSPage

import time
import string
import os

import urllib
import nstart
import config

from pyclearsilver.log import *

import neo_cgi
from pyclearsilver import handle_error
from auth import db_auth, cookieauth

class MBPage(CSPage):
    def subclassinit(self):

        hdf = self.ncgi.hdf
        proxy_path = hdf.getValue("HTTP.Soap.Action", "")
        if proxy_path and not config.gBaseURL.startswith(proxy_path):
          config.gBaseURL = proxy_path + config.gBaseURL
          config.gROSURL = proxy_path + config.gROSURL

        hdf.setValue("Config.CompressionEnabled","1")
        hdf.setValue("Config.WhiteSpaceStrip","1")

        self.login = None
        self.username = None
        self.db = None
        self.userRec = None

        now = int(time.time())
        today = time.localtime(now)
        neo_cgi.exportDate(hdf, "CGI.Today", "US/Pacific", now)

        self.authdb = db_auth.initSchema()

        via = hdf.getValue("HTTP.Via", "")
        if via:
          #if via.find(":443") != -1:
          self.http = "https://"
        
        hdf.setValue("CGI.Robot", config.get_robot_name())
        hdf.setValue("CGI.robot_type", config.get_robot_type())

        hostport_prefix = self.http + "%s%s" % (self.domain.split(':')[0], config.gROSBridgePort)
        hdf.setValue("CGI.hostport_prefix", hostport_prefix)
        hdf.setValue("CGI.ros_bridge_uri", hostport_prefix + config.gROSURL)
                     
        hdf.setValue("CGI.home_server", config.gHomeServer)
        
        self.getUsername()
        self.setStyleSheet(hdf)
        
        hdf.setValue("CGI.home_page", self.default_app_path())
        request_uri = hdf.getValue("CGI.RequestURI", "")
        if request_uri.startswith(config.gBaseURL):
            page_name = request_uri[len(config.gBaseURL):].split('?', 1)[0]
            hdf.setValue("CGI.page_name", page_name)

    def setStyleSheet(self, hdf):
        useragent = hdf.getValue("HTTP.UserAgent", "").lower()
        if useragent.find("android") != -1 or useragent.find("iphone") != -1 or useragent.find("arora") != -1:
            hdf.setValue("CGI.cur.device_style", "style_phone.css")

    def handle_actions2(self):
        hdf = self.ncgi.hdf
        hdfobj = hdf.getObj("Query.Action")
        if hdfobj:
            self.checkLoginCookie()
        CSPage.handle_actions(self)

    def getUsername(self):
        hdf = self.ncgi.hdf

        logincookie = cookieauth.parseLoginCookie(self.ncgi)
        if logincookie:
            self.username = logincookie.username

            self.userRec = self.authdb.users.lookup(self.username)

            hdf.setValue("CGI.Login", self.username)
            hdf.setValue("CGI.Login.issued_at", str(logincookie.issued_at))


    def checkLoginCookie(self):
        hdf = self.ncgi.hdf

        requestURI = hdf.getValue("CGI.RequestURI", "")

        rurl = config.gBaseURL + "login/signin0.py"

        self.authdb = db_auth.initSchema()

        logincookie = cookieauth.parseLoginCookie(self.ncgi)
        if not logincookie:
          self.redirectUri(rurl + "?q=1&request=%s" % neo_cgi.urlEscape(requestURI))

        self.username = logincookie.username
        self.userRec = self.authdb.users.lookup(self.username)
        if self.userRec:
            hdf.setValue("CGI.Role", self.userRec.role)

        if self.userRec is None or cookieauth.checkLoginCookie(self.ncgi, logincookie, self.authdb, self.username, self.userRec) == 0:
          warn("invalid cookie", rurl + "?q=1&request=%s" % neo_cgi.urlEscape(requestURI))
          self.redirectUri(rurl + "?q=1&request=%s" % neo_cgi.urlEscape(requestURI))
        # -----  the cookie is valid!!!! -------

        persist = cookieauth.getPersistCookie(hdf)
        if persist == 0:
          # reissue a new cookie with an updated timeout
          if (time.time() - logincookie.issued_at) > config.REFRESH_COOKIE_TIMEOUT:
            cookieauth.issueLoginCookie(self.ncgi, self.authdb, self.username, self.userRec.pw_hash)

        self.login = self.username

        hdf.setValue("CGI.Login", self.username)
        hdf.setValue("CGI.Login.issued_at", str(logincookie.issued_at))

        active_user = self.get_active_user()
        time_since_activity = self.get_active_user_last_activity() # seconds since they did something
        
        if active_user:
            if time_since_activity > config.ACTIVE_USER_TIMEOUT:
                # time out after one hour?
                self.remove_active_user()
            else:
                # touch the active user file
                os.utime(config.ACTIVE_USER_FILE, None)
                
        hdf.setValue("CGI.active_user", self.get_active_user())

        if self._pageparms.get("checkActive", True):
          if hdf.getValue("Cookie.inactive", "0") != "1":  
            if self.get_active_user() == "":
                self.make_active_user(hdf)
            elif not self.is_active_user():
                rurl = config.gBaseURL + "active/active.py"
                if requestURI.find("/active/") == -1:
                    self.redirectUri(rurl + "?q=1&request=%s" % neo_cgi.urlEscape(requestURI))

    def get_active_user(self):
      active_user = ""
      try:
        active_user = open(config.ACTIVE_USER_FILE, "r").read().strip()
      except:
        pass

      return active_user

    def get_active_user_last_activity(self):
      time_since_mod = 0
      try:
        modified_time = os.path.getmtime(config.ACTIVE_USER_FILE)
        time_since_mod = (time.time() - modified_time)
      except:
        pass

      return time_since_mod

    def is_active_user(self):
      if self.username == self.get_active_user():
        return True
      return False

    def make_active_user(self, hdf):
      if not os.path.exists(config.ROS_VAR_DIR):
        os.umask(0)
        os.mkdir(config.ROS_VAR_DIR, 0777)
      active_user = open(config.ACTIVE_USER_FILE, "w")
      active_user.write(self.username)
      active_user.close()

      if config.get_robot_type().startswith("texas"):
          cookie = hdf.getValue("Query.cookie", "")
          if cookie:
              cookie_file = open(config.VALID_USER_COOKIE_FILE, "w")
              cookie_file.write(cookie)
              cookie_file.close()

      if config.gLobby:
        url = config.gLobby + "/lobby/lobby/userrec.py"
        postdata = {}
        postdata['Action.MakeActiveUser'] = "1"
        postdata['active_user'] = self.username
        postdata['robot'] = config.get_robot_name()
          
        fp = urllib.urlopen(url, urllib.urlencode(postdata.items()))
        fp.read()
        fp.close()
                     

    def remove_active_user(self):
      if config.get_robot_type().startswith("texas"):
          if os.path.exists(config.VALID_USER_COOKIE_FILE):
              os.remove(config.VALID_USER_COOKIE_FILE)

      if os.path.exists(config.ACTIVE_USER_FILE):
        os.remove(config.ACTIVE_USER_FILE)

        if config.gLobby:
          url = config.gLobby + "/lobby/lobby/userrec.py"
          postdata = {}
          postdata['Action.RemoveActiveUser'] = "1"
          postdata['robot'] = config.get_robot_name()

          fp = urllib.urlopen(url, urllib.urlencode(postdata.items()))
          fp.read()
          fp.close()

          

    def default_app_path(self):
      return self.http + self.ncgi.hdf.getValue("HTTP.Host", "") + config.gBaseURL + "%s/" % config.gDefaultModule

    def close(self):
      if hasattr(self, "db") and self.db:
        self.db.close()
        self.db = None
      if hasattr(self, "authdb") and self.authdb:
        self.authdb.close()
        self.authdb = None

    def __del__(self):
      self.close()


