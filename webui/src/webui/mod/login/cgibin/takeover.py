#!/usr/bin/env python

import nstart
import config
import os, sys, string, time
import socket

from pyclearsilver.log import *

from pyclearsilver.CSPage import Context
import neo_cgi, neo_cs, neo_util
from MBPage import MBPage
from webui import config

from auth import browserauth
from auth import cookieauth
from auth import db_auth
from auth import pwauth

TAKEOVER_FILE = os.path.join(config.ROS_VAR_DIR, "takeover.dat")

class SignInPage(MBPage):
    def setup(self, hdf):
        pass
    
    def display(self, hdf):
        username = hdf.getValue("Query.username", "")
        if not username: return

        ip_address = socket.gethostbyname(config.gLobbyHost)

        if hdf.getValue("CGI.RemoteAddress", "") != ip_address:
            warn("request from lobby on %s does not match %s" % (hdf.getValue("CGI.RemoteAddress", ""), ip_address))
            #return
        else:
            warn("ip address is authorized")

        if not os.path.exists(config.ROS_VAR_DIR):
            os.umask(0)
            os.mkdir(config.ROS_VAR_DIR, 0777)
        takeover = open(TAKEOVER_FILE, "w")
        takeover.write(username)
        takeover.close()


    def Action_Response(self, hdf):
        if os.path.exists(TAKEOVER_FILE):
            os.remove(TAKEOVER_FILE)

        if hdf.getValue("Query.Action.Response", "") == "allow":
            self.redirectUri(config.gBaseURL + "login/signin.py?Action.Logout=1")
        elif hdf.getValue("Query.Action.Response", "") == "deny":
            self.redirectUri(config.gBaseURL + "app/texas_web_teleop/texas_web_teleop_mini/")

            


def run(context):
    page = SignInPage(context, pagename="takeover", nologin=1)
    return page

def main(context):
  page = run(context)
  page.start()
  

if __name__ == "__main__":
    main(Context())

