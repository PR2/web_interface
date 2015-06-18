#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""


import os, sys, string, time, getopt
from log import *

import types

import odb
#import sqliterep as sqlite
import sqlite3
sqlite = sqlite3

import _sqlite3
odb.OperationalError = _sqlite3.OperationalError
odb.DatabaseError = _sqlite3.DatabaseError
odb.DataError = _sqlite3.DataError
odb.IntegrityError = _sqlite3.IntegrityError
odb.NotSupportedError = _sqlite3.NotSupportedError

class Cursor(odb.Cursor):
  def insert_id(self, tablename, colname):
    return self.cursor.lastrowid

  def execute(self, sql):
    try:
      return self.cursor.execute(sql)
    except _sqlite3.DatabaseError, reason:
      reason = str(reason) + "(%s)" % sql
      raise _sqlite3.DatabaseError, reason
    
      

  def begin(self):
    pass


class Connection(odb.Connection):
  def __init__(self, *args, **kwargs):
    odb.Connection.__init__(self)

    try:
      self._conn = apply(sqlite.connect, args, kwargs)
    except:
      warn("unable to open db", args)
      raise

    self.SQLError = sqlite.Error

  def getConnType(self): return "sqlite"
    
  def cursor(self):
    return Cursor(self._conn.cursor())

  def encode(self, str):
    return sqlite3.encode(str)
  def decode(self, str):
    return sqlite3.decode(str)

  def escape(self,s):
    if s is None:
      return None
    elif type(s) == types.StringType:
      return string.replace(s,"'","''")
    elif type(s) in (types.IntType, types.FloatType):
      return s
    elif type(s) == types.UnicodeType:
      return str(s)
    else:
      warn("unknown column data type: <%s> value=%s" % (type(s), s[:100]))
      return str(s)


  def listTables(self, cursor):
    cursor.execute("select name from sqlite_master where type='table'")
    rows = cursor.fetchall()
    tables = []
    for row in rows: tables.append(row[0])
    return tables

  def supportsTriggers(self): return True

  def listTriggers(self, cursor):
    cursor.execute("select name from sqlite_master where type='trigger'")
    rows = cursor.fetchall()
    tables = []
    for row in rows: tables.append(row[0])
    return tables

  def listIndices(self, tableName, cursor):
    cursor.execute("select name from sqlite_master where type='index'")
    rows = cursor.fetchall()
    tables = []
    for row in rows: 
      if row[0].find("sqlite_autoindex_") != -1:
        continue
      tables.append(row[0])
    return tables

  def listFieldsDict(self, table_name, cursor):
    sql = "pragma table_info(%s)" % table_name
    cursor.execute(sql)
    rows = cursor.fetchall()

    columns = {}
    for row in rows:
      colname = row[1]
      columns[colname] = row
    return columns

  def _tableCreateStatement(self, table_name, cursor):
    sql = "select sql from sqlite_master where type='table' and name='%s'" % table_name
    print sql
    cursor.execute(sql)
    row = cursor.fetchone()
    sqlstatement = row[0]
    return sqlstatement
    

  def alterTableToMatch(self, table, cursor):
    tableName = table.getTableName()
    debug("alterTableToMatch", tableName)
    tmpTableName = tableName + "_" + str(os.getpid())

    invalidAppCols, invalidDBCols = table.checkTable(warnflag=0)
#    warn(invalidAppCols, invalidDBCols)
         

##     if invalidAppCols or invalidDBCols:
##       return

    if not invalidAppCols and not invalidDBCols:
      return


    oldcols = self.listFieldsDict(tableName, cursor)
#    tmpcols = oldcols.keys()
    
    tmpcols = []
    newcols = table.getAppColumnList()
    for colname, coltype, options in newcols:
      if oldcols.has_key(colname): tmpcols.append(colname)
    
    tmpcolnames = string.join(tmpcols, ",")

    count = table.fetchRowCount()
    warn("count for %s=" % table.getTableName(), count)
      
    statements = []

    #sql = "begin transaction"
    #statements.append(sql)

#    sql = "create temporary table %s (%s)" % (tmpTableName, tmpcolnames)
    sql = "create table %s (%s)" % (tmpTableName, tmpcolnames)
    statements.append(sql)

    sql = "insert into %s select %s from %s" % (tmpTableName, tmpcolnames, tableName)
    statements.append(sql)

    sql = "drop table %s" % tableName
    statements.append(sql)
    
    sql = table._createTableSQL()
    statements.append(sql)

    sql = "insert into %s(%s) select %s from %s" % (tableName, tmpcolnames, tmpcolnames, tmpTableName)
    statements.append(sql)

    sql = "drop table %s" % tmpTableName
    statements.append(sql)
    
    #sql = "commit"
    #statements.append(sql)

    self.begin()
    for statement in statements:
      print statement
      cursor.execute(statement)
    self.commit()

  def auto_increment(self, coltype):
    return coltype, ""

  def create_fullTextSearchTable(self, tableName, column_list):
    defs = []
    for colname, coltype, options in column_list:
      if colname in ("rowid", "docid"): continue
      defs.append(colname)
    defs = string.join(defs, ", ")
    
    return "CREATE virtual TABLE %s using FTS3(%s)" % (tableName, defs)

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
