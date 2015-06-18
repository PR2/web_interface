#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""


import os, sys, string, time, getopt
from pyclearsilver.log import *

from pyclearsilver import odb

import config

#import fcrypt as crypt
import crypt

def _createCheckVal(username, issued_at, pw_hash, vcode):
  checkval = "%s:%s" % (username, now)
  realcheckval = "%s:%s:%s" % (checkval, pw_hash, vcode)
  checkval_hash = crypt.crypt(realcheckval,config.gAuthSalt)
  return checkval, checkval_hash


# -------------------------------
# issueLoginCookie
#
# format: "login:issued_at_time_t:hash(pw_hash+issued_at_time_t)"
# ex: "V1/jeske:2123123:AS132dd12"

def generateBrowserCookie(authdb, ipaddr):
  now = int(time.time())

  row = authdb.browserid.newRow()
  row.creationDate = now
  row.ipaddr = ipaddr
  row.save()

  cookie = "V1/%09d" % row.browserid

  return cookie, row.browserid

def issueBrowserCookie(ncgi, authdb, domain):
  ipaddr = ncgi.hdf.getValue("CGI.RemoteAddress", "")

  bcookie, browserid = generateBrowserCookie(authdb, ipaddr)
  ncgi.cookieSet("MB_B", bcookie, persist=1, path="/", domain=config.gDomain)
  return browserid
  

def clearBrowserCookie(ncgi):
  ncgi.cookieClear("MB_B", "", "/")

def getBrowserCookie(ncgi):
  bcookie = ncgi.hdf.getValue("Cookie.MB_B","")

  if not bcookie: return None

  version, restCookie = string.split(bcookie, "/", 1)
  browserid = int(restCookie)

  return browserid
  

def _checkBrowserCookie(authdb, cookie, ipaddr):
  version, restCookie = string.split(cookie, "/", 1)
  if version != "V1":
    warn("browserauth.py", "invalid browser cookie, version", version, cookie)
    return None

  browserid = int(restCookie)

  try:
    row = authdb.browserid.fetchRow(("browserid", browserid))
  except odb.eNoMatchingRows:
    warn("browserauth.py", "invalid browser cookie, browserid not found")
    return browserid
#    return None

  if row.ipaddr != ipaddr:
    warn("browserauth.py", "ipaddr mismatch", row.ipaddr, ipaddr)

  debug("browserauth.py", "cookie", browserid)

  return browserid

def checkBrowserCookie(authdb, ncgi):
  bcookie = ncgi.hdf.getValue("Cookie.MB_B","")
  if not bcookie: return None

  ipaddr = ncgi.hdf.getValue("CGI.RemoteAddress", "")
  
  browserid = _checkBrowserCookie(authdb, bcookie, ipaddr)
  return browserid
  
  

def test():
  pass

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug"])

  testflag = 0
  if len(args) == 0:
    usage(progname)
    return
  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1

  if testflag:
    test()
    return


if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
