#!/usr/bin/env python 

import nstart
import config
import sys
from pyclearsilver.CSPage import Context
from MBPage import MBPage

import MinibooMailbox

class IndexPage(MBPage):
    def display(self):
        hdf = self.ncgi.hdf

        if hdf.getValue("Query.st","") == "3":
            self.handle_stage_3()

    def handle_stage_3(self):
        hdf = self.ncgi.hdf
        self.pagename = "forgotpw_3"

        q_login = hdf.getValue("Query.login","")

        hdf.setValue("CGI.Login",q_login)

    def Action_ResetPw(self):
        hdf = self.ncgi.hdf
        q_login = hdf.getValue("Query.login","")
        if q_login:
            self.redirectUri("forgotpw.py?st=3&login=%s" % q_login)

    def Action_SetPassword(self):
        hdf = self.ncgi.hdf
        q_login = hdf.getValue("Query.login","")
        q_pw1 = hdf.getValue("Query.pw1","")
        q_pw2 = hdf.getValue("Query.pw2","")

        if not q_login:
            self.redirectUri("forgotpw.py?q=1")

        if not q_pw1 or (q_pw1 != q_pw2):
            self.redirectUri("forgotpw.py?st=3&login=%s&err=pwdontmatch" % q_login)

        MB = MinibooMailbox.loadMailbox(q_login)
        MB.setPassword(q_pw1)
        self.MB = MB
        self.issueLoginCookie(q_login,self.registrydb.getOption("pw_hash"))
        self.redirectUri(config.gBaseURL + q_login + "/mail/index.py")
        

def run(context):
  return IndexPage(context, pagename="forgotpw_1",nologin=1)

def main(context):
  run(context).start()

if __name__ == "__main__":
    main(Context())
