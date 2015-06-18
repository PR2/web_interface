#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import config

import string
import subprocess
from pyclearsilver.log import *

import MBPage
import webutil


class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    pass
    
  def display(self, hdf):
    webutil.grabTopics(hdf, [])
    webutil.set_tabs(hdf, ["status", "processes"])    
    try:
      proc = subprocess.Popen(["sudo", "ckill", "list"], stdout=subprocess.PIPE)
      out, err = proc.communicate()
      out = string.strip(out)
      i = 0
      lines = out.split('\n')
      if out == "" and hdf.getValue("Query.reset", "") != "":
        self.redirectUri(config.gBaseURL + "webui/reset.py?Action.DoReset=1")
      else:
        for line in lines:
          hdf.setValue("CGI.cur.lines.%d" % i, line)
          i += 1

    except:
      hdf.setValue("CGI.cur.error", "Unable to call ckill on this robot.")
          
def run(context):
  return MyPage(context, pagename="processes", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
