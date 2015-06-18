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

from web_msgs.msg import WebEvent
import rospy

class SignInPage(MBPage):
    def setup(self, hdf):
        pass
    
    def display(self, hdf):
        username = hdf.getValue("Query.username", "")
        if not username: return

        # need to check ip address is coming from the lobby
        # lobby address is in config
        ip_address = socket.gethostbyname(config.gLobbyHost)

        if hdf.getValue("CGI.RemoteAddress", "") != ip_address:
            warn("request from lobby on %s does not match %s" % (hdf.getValue("CGI.RemoteAddress", ""), ip_address))
            return
        else:
            warn("ip address is authorized")

        # create the user record and transfer all info
        user = self.authdb.users.lookupCreate(username=username)

        for key in ("role", "pw_hash", "status", "dismissed_notices", "favorite_apps", "skype_id"):
            user[key] = hdf.getValue("Query." + key, "")

        user.save()

        # write the active user file
        self.username = username
        self.make_active_user(hdf)

        # publish a web event that we logged in
        pub = rospy.Publisher("/webui/events", WebEvent)
        rospy.init_node("webui_login", anonymous=True)
        msg = WebEvent()
        msg.source = "user"
        msg.type = "login (lobby)"
        msg.data = self.username
        pub.publish(msg)


def run(context):
    page = SignInPage(context, pagename="signin_lobby", nologin=1)
    return page

def main(context):
  page = run(context)
  page.start()
  

if __name__ == "__main__":
    main(Context())

