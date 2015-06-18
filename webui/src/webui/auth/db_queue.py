#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import nstart
import os, sys, string, time, getopt

from pyclearsilver.log import *

import config

from pyclearsilver import odb, hdfhelp, odb_sqlite3
from pyclearsilver import CSPage

from pyclearsilver.odb import *

gDBPath = "host"
gDBFilename = "queue"
gDBTablePrefix = "queue"

class QueueDB(odb.Database):
  def __init__(self, conn, debug=0):
    odb.Database.__init__(self, conn, debug=debug)

    self.addTable("queue", gDBTablePrefix + "_queue", QueueTable,
		  rowClass=CommandRecord)

  def defaultRowClass(self):
    return hdfhelp.HdfRow
  def defaultRowListClass(self):
    return hdfhelp.HdfItemList

class QueueTable(odb.Table):
  def _defineRows(self):
    self.d_addColumn("qid",kInteger,None,primarykey = 1,
                     autoincrement = 1)

    self.d_addColumn("username",kVarString)
    self.d_addColumn("cmd",kVarString, indexed=1)
    self.d_addColumn("data",kVarString)
    self.d_addColumn("startDate", kInteger, default=0)  
    ## when to activate the command

  def getCommands(self, cmd, when=None):
    if when:
      rows = self.fetchRows(('cmd', cmd), where="startDate <= %s" % when)
    else:
      rows = self.fetchRows(('cmd', cmd))
    return rows

  def newCommand(self, username, cmd, startDate, data=""):
    row = self.newRow()
    row.username = username
    row.startDate = startDate
    row.cmd = cmd
    row.data = data
    row.save()

    return row
    
class CommandRecord(odb.Row):
  pass
    

def fullDBPath(path_to_store):
  return os.path.join(path_to_store, gDBFilename + ".db3")

def initSchema(create=0, timeout=None):
  if timeout is None: timeout = 600

  path = config.getSiteDBPath("host")

  if create == 1:
    config.createDBPath(path)

  conn = odb_sqlite3.Connection(fullDBPath(path),
                              autocommit=0,
                              timeout=timeout)

  db = QueueDB(conn,debug=debug)

  if create:
    db.createTables()
    db.synchronizeSchema()
    db.createIndices()

    if config.gWebUserID is not None and config.gWebGroupID is not None:
      config.webChown(fullDBPath(path))

  return db

def exists():
  path = config.getSiteDBPath("host")
  fn = fullDBPath(path)
  if os.path.exists(fn): 
    return 1
  return 0
  

def createDB():
  db = initSchema(create=1)
  return db
  
  
def test():
  db = initSchema()

  rows = db.queue.fetchAllRows()
  for row in rows:
    print row.username, row.cmd, row.data


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

  
  
