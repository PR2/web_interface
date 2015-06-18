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

import webutil
from ros import rosservice

class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    pass
    
  def display(self, hdf):
    webutil.grabTopics(hdf, [])
    webutil.set_tabs(hdf, ["status", "services"])

    services = rosservice.get_service_list(None)
    services.sort()
    
    i = 0
    for service in services:
      i = i + 1
      hdf.setValue("CGI.cur.services.%d" % i, service)
    
    

def run(context):
  return MyPage(context, pagename="services", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
