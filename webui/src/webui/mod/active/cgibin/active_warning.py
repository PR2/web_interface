#!/usr/bin/env python

import nstart
import config
import os, sys, string, time

from pyclearsilver.log import *

from pyclearsilver.CSPage import Context
import neo_cgi, neo_cs, neo_util
from MBPage import MBPage
import webutil

class ActivePage(MBPage):
    def setup(self, hdf):
      self.requestURI = hdf.getValue("Query.request", "")
      if not self.requestURI:
        self.requestURI = self.default_app_path()

    def display(self, hdf):
      webutil.grabTopics(hdf, [])
      pass

    def __del__(self):
      if self.authdb:
        self.authdb.close()
        self.authdb = None

def run(context):
    page = ActivePage(context, pagename="active_warning", nologin=False)
    return page

def main(context):
  page = run(context)
  page.start()
  

if __name__ == "__main__":
    main(Context())
