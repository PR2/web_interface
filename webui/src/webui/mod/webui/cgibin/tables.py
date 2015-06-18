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

from pyclearsilver import scaffold

class MyPage(scaffold.DBScaffold):
  def setup(self, hdf):
    self.db = db_webui.initSchema()
    scaffold.DBScaffold.setupDetails(self, self.db)
    scaffold.DBScaffold.setup(self, hdf)
    
def run(context):
  return MyPage(context, pagename="default")

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
