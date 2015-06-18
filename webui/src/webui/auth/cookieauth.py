#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""


import os, sys, string, time, getopt
from pyclearsilver.log import *

#import fcrypt as crypt
import crypt

import config

import browserauth

def _createCheckVal(username, issued_at, pw_hash, vcode):
  now = int(time.time())
  checkval = "%s:%s" % (username, now)
  realcheckval = "%s:%s:%s" % (checkval, pw_hash, vcode)
  checkval_hash = crypt.crypt(realcheckval,config.gAuthSalt)
  checkval_hash = checkval_hash[2:]
  return checkval, checkval_hash


# -------------------------------
# issueLoginCookie
#
# format: "login:issued_at_time_t:hash(pw_hash+issued_at_time_t)"
# ex: "V1/jeske:2123123:AS132dd12"

def generateCookie(username, pw_hash):
  now = int(time.time())
  checkval, checkval_hash = _createCheckVal(username, now, pw_hash, config.gAuthVCode)
  cookie = "V1/%s=%s" % (checkval,checkval_hash)

  return cookie

def getDomain(hdf):
  hostname = hdf.getValue("HTTP.Host", "")
  parts = hostname.split(":", 1)
  hostname = parts[0]
  if hostname[-1] in string.digits:  ## if this is an IP address
    return hostname 
  parts = string.split(hostname, ".")
  domain = string.join(parts[1:], ".")
  return domain

def getPersistCookie(hdf):
  try:
    persist = hdf.getIntValue("Cookie.MB_persist", 0)
  except:
    persist = 0
  return persist

def setPersistCookie(ncgi, persist):
  ncgi.cookieSet("MB_persist", persist, persist=1, domain=config.gDomain)  


def issueLoginCookie(ncgi, authdb, username, pw_hash, persist=None):
  if persist == None:
    persist = getPersistCookie(ncgi.hdf)

  domain = getDomain(ncgi.hdf)

  browserid = browserauth.checkBrowserCookie(authdb, ncgi)
  if browserid is None:
    # set the browser cookie
    browserid = browserauth.issueBrowserCookie(ncgi, authdb, domain)

  debug("cookieauth.py", "BrowserID", browserid)
  debug("cookieauth.py", "domain", domain)

  if persist == 1:
    t = time.time()
    t = t + (86400*14)
    timestr = time.strftime("%A, %d-%b-%Y %H:%M:%S GMT", time.localtime(t))
  else:
    timestr = ""

  cookie = generateCookie(username, pw_hash)
#  ncgi.cookieSet("MB_L1", cookie, persist=persist, path=config.gBaseURL, domain=domain, time_str=timestr)
  ncgi.cookieSet("MB_L1", cookie, persist=persist, path=config.gBaseURL, time_str=timestr)

  #warn("cookieauth.py", "Issued login cookie", username,cookie, domain, timestr, persist)


def clearLoginCookie(ncgi, username, domain=None):
  domain = getDomain(ncgi.hdf)
  ncgi.cookieClear("MB_L1", "", config.gBaseURL)
  ncgi.cookieClear("MB_L1", "", "/")
  if domain:
    ncgi.cookieClear("MB_L1", domain, config.gBaseURL)
    ncgi.cookieClear("MB_L1", domain, "/")
    

class LoginCookie:
  def __init__(self):
    self.username = None
    self.issued_at = None
    self.checkval_hash = None
    self.cookie = None

def parseLoginCookie(ncgi):
  cookie = ncgi.hdf.getValue("Cookie.MB_L1","")
  if not cookie:
    #warn("cookieauth.py", "no cookie!")
    return 0

  version, restCookie = string.split(cookie, "/", 1)
  if version != "V1":
    warn("cookieauth.py", "invalid cookie, version", version, cookie)
    return 0
  checkval,checkval_hash = string.split(restCookie,"=", 1)
  username,issued_at = string.split(checkval,":")

  cookie = LoginCookie()
  cookie.cookie = cookie
  cookie.username = username
  cookie.issued_at = int(issued_at)
  cookie.checkval_hash = checkval_hash

  return cookie



def checkLoginCookie(ncgi, logincookie, authdb, username, userRec):

  if username != logincookie.username:
    warn("cookieauth.py", "invalid cookie, username mismatch", username, logincookie.username)
    return 0

  persist = getPersistCookie(ncgi.hdf)

  # check for timeout
  if persist == 0:
    if (time.time() - logincookie.issued_at) > config.LOGIN_TIMEOUT:
      warn("cookieauth.py", "invalid cookie, timeout", logincookie.issued_at)
      return 0

  pw_hash = userRec.pw_hash

  #warn("cookieauth.py", "cookie", username, logincookie.issued_at, pw_hash, logincookie.checkval_hash)

  v_checkval, v_checkval_hash = _createCheckVal(username, logincookie.issued_at, pw_hash, config.gAuthVCode)

  if logincookie.checkval_hash != v_checkval_hash:
    warn("cookieauth.py", "checkval mismatch", logincookie.checkval_hash, v_checkval_hash)

  return 1






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
