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
import nodeutil

class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    pass
    
  def display(self, hdf):
    topic = hdf.getValue("Query.topic", "")
    hdf.setValue("CGI.cur.topic", topic)
    webutil.set_tabs(hdf, ["status", "topics"])
    webutil.grabTopics(hdf, ["/topics", topic, "/battery_state", "/power_board_state", "/app_status"])

    topic_data = nodeutil.topic_info(topic)

    if "error" in topic_data:
      hdf.setValue("CGI.cur.error", topic_data["error"])
    else:
      webutil.hdf_array(hdf, sorted(topic_data["publishers"]), "CGI.cur.publishers")
      webutil.hdf_array(hdf, sorted(topic_data["subscribers"]), "CGI.cur.subscribers")

def run(context):
  return MyPage(context, pagename="topic", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
