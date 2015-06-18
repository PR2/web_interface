#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import nstart
import os, sys, string, time, getopt
import urllib

from pyclearsilver.log import *

import config

from pyclearsilver import odb, hdfhelp, odb_sqlite3
from pyclearsilver import CSPage

from pyclearsilver.odb import *

import pwauth

gDBSubPath = "host"
gDBFilename = "auth"
gDBTablePrefix = "auth"

class AuthDB(odb.Database):
  def __init__(self,db,debug=0):
    odb.Database.__init__(self, db, debug=debug)

    self.addTable("users", gDBTablePrefix + "_users", UserTable,
		  rowClass=UserRecord)
    self.addTable("login", gDBTablePrefix + "_login", UserLoginTable)
    self.addTable("vcode", gDBTablePrefix + "_vcode", VCodeTable)
    self.addTable("browserid", gDBTablePrefix + "_browserid", BrowserTable)

  def defaultRowClass(self):
    return hdfhelp.HdfRow
  def defaultRowListClass(self):
    return hdfhelp.HdfItemList

  def getAllUsers(self):
    users = []
    rows = self.users.fetchAllRows()
    for row in rows:
      users.append(row.username)

    return users
    



class UserTable(odb.Table):
  def _defineRows(self):
    self.d_addColumn("uid",kInteger,None,primarykey = 1,
                     autoincrement = 1)

    self.d_addColumn("username",kVarString, indexed=1, unique=1)
    self.d_addColumn("role", kVarString, default="")
    self.d_addColumn("pw_hash",kVarString)
    self.d_addColumn("status",kInteger, default=0)
    ## status == 1: disabled user
    self.d_addColumn("creationDate", kInteger, default=0)
    self.d_addColumn("dismissed_notices", kVarString, default="")
    self.d_addColumn("favorite_apps", kVarString, default="")
    self.d_addColumn("changePassword",kInteger, default=0)
    self.d_addColumn("skype_id", kVarString, default="") 

  def lookup(self, username):
    try:
      row = self.fetchRow(('username', username))
    except odb.eNoMatchingRows, reason:
      row = None
    return row

  def new(self, username, password):
    row = self.lookup(username)
    if row is not None: return row

    row = self.newRow()
    row.username = username
    row.creationDate = int(time.time())
    row.setPassword(password)
    row.save()

    return row
    
class UserRecord(hdfhelp.HdfRow):
  def checkPasswordHash(self, passwordHash):
    if len(self.pw_hash) < 2: return 0
    if passwordHash == self.pw_hash: return 1
    return 0


  def checkPassword(self, password):
    if len(self.pw_hash) < 2: return 0

    return pwauth.checkPassword(password, self.pw_hash)

  def setPassword(self, new_password):
    self.pw_hash = pwauth.cryptPassword(new_password)
    self.changePassword = 0
    self.save()
    
  def favorite_apps_list(self):
    return self.favorite_apps.split(',') if len(self.favorite_apps) > 0 else []
    
  def is_favorite_app(self, app_taskid):
    return self.favorite_apps.find(app_taskid) != -1
    
  def add_favorite_app(self, app_taskid):
    apps = self.favorite_apps_list()
    if not app_taskid in apps:
      apps.append(app_taskid)
      self.favorite_apps = string.join(apps, ',')
      self.save()  
      self.sync_with_lobby()

  def remove_favorite_app(self, app_taskid):
    apps = self.favorite_apps_list()
    if app_taskid in apps:
      apps.remove(app_taskid)
      self.favorite_apps = string.join(apps, ',')
      self.save()        
      self.sync_with_lobby()

  def dismissed_notice_list(self):
    return self.dismissed_notices.split(',') if len(self.dismissed_notices) > 0 else []

  def notice_dismissed(self, notice):
    return self.dismissed_notices.find(notice) != -1

  def dismiss_notice(self, notice):
    notices = self.dismissed_notice_list()
    if not notice in notices:
      notices.append(notice)
      self.dismissed_notices = string.join(notices, ',')
      self.save()
      self.sync_with_lobby()

  def set_skype_id(self, id):
    self.skype_id = id
    self.save()

    self.sync_with_lobby()

  def sync_with_lobby(self):
    if not config.gLobby: return

    ## post to the lobby
    url = config.gLobby + "/lobby/lobby/userrec.py"
    postdata = {}
    postdata['username'] = self.username

    postdata['skype_id'] = self.skype_id
    postdata['dismissed_notices'] = self.dismissed_notices
    postdata['favorite_apps'] = self.favorite_apps

    fp = urllib.urlopen(url, urllib.urlencode(postdata.items()))
    fp.read()
    fp.close()
                                              

class UserLoginTable(odb.Table):
  def _defineRows(self):
    self.d_addColumn("uid",kInteger, primarykey=1)
    self.d_addColumn("username",kVarString, indexed=1, primarykey=1)
    self.d_addColumn("time", kCreatedStampMS, primarykey=1)

    self.d_addColumn("loginType", kInteger)   
    # 0 - incorrect password
    # 1 - correct password

    self.d_addColumn("browserid",kVarString)
    self.d_addColumn("ipaddr",kVarString)


class VCodeTable(odb.Table):
  def _defineRows(self):
    self.d_addColumn("username",kVarString, primarykey=1)
    self.d_addColumn("vcode",kInteger, default=0)
    self.d_addColumn("browserid",kInteger, default=0)
    self.d_addColumn("creationDate", kInteger, default=0)


class BrowserTable(odb.Table):
  def _defineRows(self):
    self.d_addColumn("browserid",kInteger, primarykey=1, autoincrement=1)
    self.d_addColumn("ipaddr", kVarString)
    self.d_addColumn("creationDate", kInteger, default=0)

    

def fullDBPath(path_to_store):
  return os.path.join(path_to_store, gDBFilename + ".db3")

def initSchema(create=0, timeout=None):
  if timeout is None: timeout = 600

  path = config.getSiteDBPath(gDBSubPath)

  if create == 1:
    config.createDBPath(path)

  conn = odb_sqlite3.Connection(fullDBPath(path),
                               timeout=timeout)

  db = AuthDB(conn,debug=debug)

  if create:
    db.createTables()
    db.synchronizeSchema()
    db.createIndices()

    if 0:
      if config.gWebUserID is not None and config.gWebGroupID is not None:
        config.webChown(fullDBPath(path))

  return db

def exists(username):
  path = config.getSiteDBPath(gDBSubPath)
  fn = fullDBPath(path)
  if os.path.exists(fn): 
    return 1
  return 0
  

def createDB():
  db = initSchema(create=1)
  return db
  
  
def test():
  db = initSchema()

  rows = db.users.fetchAllRows()
  for row in rows:
    print row.username, row.pw_hash



def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug"])

  testflag = 0
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

  db = initSchema(create=1)



if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)

  
  
