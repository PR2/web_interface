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

from roslib.names import ns_join, get_ros_namespace, make_caller_id, make_global_ns, GLOBALNS


def _get_caller_id():
    return make_caller_id('rosparam-%s'%os.getpid())

class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    self.master = roslib.scriptutil.get_master()
    
  def display(self, hdf):
    webutil.grabTopics(hdf, [])
    webutil.set_tabs(hdf, ["status", "params"])    

    _c, _t, params = self.master.getParam(_get_caller_id(), "/")
    self.flatten(hdf, params, path="CGI.cur.params.")
    
  def flatten(self, hdf, params, path=""):
    for k,v in params.items():
      _path=path + k
      if type(v) == type({}):
        self.flatten(hdf, v, _path + ".")
      else:
        hdf.setValue(_path, str(v)[:128])
        

  def Action_Edit(self, hdf):
    key = hdf.getValue("Query.element_id", "")
    value = hdf.getValue("Query.update_value", "")
    delete = hdf.getValue("Query.delete", "")
    hdf.setValue("CGI.result", hdf.getValue("Query.original_html", "ERROR"))
        
    if delete == "1":
      results = self.master.deleteParam(_get_caller_id(), key)
      if results[0] == 1:
        hdf.setValue("CGI.result", "OK")
      else:
        hdf.setValue("CGI.result", "")
    elif key != "" and value != "":
      results = self.master.setParam(_get_caller_id(), key, value)
      if results[0] == 1:
        hdf.setValue("CGI.result", value)
    
    hdf.setValue("Content", "ajax_result.cs")
    
  def Action_New(self, hdf):
    key = hdf.getValue("Query.key", "")
    value = hdf.getValue("Query.value", "")

    if key != "" and value != "":
      results = self.master.setParam(_get_caller_id(), key, value)
      if results[0] == 1:
        hdf.setValue("CGI.result", value)
        self.redirectUri("params.py")
    

def run(context):
  return MyPage(context, pagename="params", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
