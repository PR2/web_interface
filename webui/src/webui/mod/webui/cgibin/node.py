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
    node = hdf.getValue("Query.node", "")
    hdf.setValue("CGI.cur.node", node)
    webutil.set_tabs(hdf, ["status", "nodes"])
    webutil.grabTopics(hdf, ["/topics"])

    node_data = nodeutil.node_info(node)

    webutil.hdf_array(hdf, sorted(node_data["subscriptions"]), "CGI.cur.subscriptions")
    webutil.hdf_array(hdf, sorted(node_data["publications"]), "CGI.cur.publications")
    webutil.hdf_array(hdf, sorted(node_data["services"]), "CGI.cur.services")
    
    if "error" in node_data:
      hdf.setValue("CGI.cur.error", node_data["error"])
    
def run(context):
  return MyPage(context, pagename="node", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
