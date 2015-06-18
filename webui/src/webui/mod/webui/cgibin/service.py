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
#    webutil.grabTopics(hdf, [])
    webutil.set_tabs(hdf, ["status", "services"])

    service = hdf.getValue("Query.service", "")

    args = rosservice.get_service_args(service)
    args = args.strip()
    args = args.split(" ")

    hdf.setValue("CGI.cur.service", service)

    i = 0
    for arg in args:
      i = i + 1
      hdf.setValue("CGI.cur.args.%d" % i, arg)
    
    

def run(context):
  return MyPage(context, pagename="service", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
