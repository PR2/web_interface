#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import nstart
import config

import os, sys, string, time, getopt, re
import subprocess
from pyclearsilver.log import *

import neo_cgi, neo_util, neo_cs

from pyclearsilver import CSPage
from pyclearsilver import odb

import MBPage
import db_webui

import webutil


class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    pass
    
  def display(self, hdf):
    webutil.grabTopics(hdf, [])
    
  def Action_DoReset(self, hdf):
    # do rosreset here -- rosreset init script, webui init script
    warn("calling robot reset")
    proc = subprocess.Popen(["sudo", "robot", "reset", "--force", "-u", self.username], stdout=subprocess.PIPE)
    out, err = proc.communicate()
    if out:
      warn("out: " + out)
    if err:
      warn("err: " + err)
    
    time.sleep(5)    
    warn("starting webui")
        
    self.redirectUri(config.gBaseURL + "webui/reset.py")

  def Action_AjaxReset(self, hdf):
    # do rosreset here -- rosreset init script, webui init script
    warn("calling ajax robot reset")
    proc = subprocess.Popen(["sudo", "robot", "reset", "--force", "-u", self.username], stdout=subprocess.PIPE)
    out, err = proc.communicate()
    if out:
      warn("out: " + out)
    if err:
      warn("err: " + err)

    time.sleep(10)    
    warn("starting webui")

    hdf.setValue("CGI.result", "OK")

    hdf.setValue("Content", "ajax_result.cs")


def run(context):
  return MyPage(context, pagename="reset", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()

