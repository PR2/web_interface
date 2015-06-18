#! /usr/bin/env python

import os, sys, string, time, getopt
from log import *

import neo_cgi, neo_cs, neo_util
import odb, hdfhelp, CSPage

class Scaffold(CSPage.CSPage):
  def setupDetails(self, db, tableName):
    self.db = db
    self.tableName = tableName

  def display(self, hdf):
    pathinfo = hdf.getValue("CGI.PathInfo", "")

    pathparts = []
    methodName = "list"
    if pathinfo: 
      pathparts = string.split(pathinfo, "/")

      if len(pathparts):
        methodName = pathparts[0]

    self.activate(hdf, methodName)

  def activate(self, hdf, methodName):
    modpath, f = os.path.split(os.path.abspath(__file__) )
    templatePath = os.path.join(modpath, "templates")
    ##self.clearPaths()
    #warn(templatePath)
    self.setPaths([templatePath])

    method = getattr(self, methodName)

    self.pagename = methodName

    method(hdf)

  def list(self, hdf):
    hdf.setValue("CGI.cur.title", "List %s Table" % self.tableName)

    self.__listTable(hdf, self.tableName, "CGI.cur.table")


  def __listTable(self, hdf, tableName, prefix):
    tbl = self.db[tableName]
    tableColumnlist = tbl.getColumnList()

    columnlist = []
    joins = []
    for col, d, colinfo in tableColumnlist:
      if colinfo.has_key("foreign_table"):
        foreign_table = colinfo["foreign_table"]
        joins.append(col)
        columnlist.append(("_" + col + "." + "name", d, colinfo))
      else:
        columnlist.append((col, d, colinfo))
    rows = tbl.fetchRows(join2=joins)
#    rows = tbl.fetchRows()
    self.__exportColumnHeaders(columnlist, hdf, prefix)
    primaryKeys = tbl.getPrimaryKeyList()

    n = 0
    for row in rows:
      n = n + 1
      aprefix = prefix + ".rows.%s" % n
      self.__exportRow(row, columnlist, hdf, aprefix, primaryKeys)

  def __exportColumnHeaders(self, columnlist, hdf, prefix):
    for col_name,col_type,options in columnlist:
      col_name = col_name.replace(".", "_")
      if options.get("no_export", 0): continue
      aprefix = prefix + ".fields.%s" % col_name
      hdf.setValue(aprefix + ".name", col_name)

      hdf.setValue(aprefix + ".col_type", col_type.odbType())
      hdfhelp.hdfExportDict(aprefix + ".options", hdf, options)
      

  def __exportRow(self, row, columnlist, hdf, prefix, primaryKeys):

    for col_name,col_type,options in columnlist:
      if options.get("no_export", 0): continue

      a_col_name = col_name.replace(".", "_")
      nprefix = prefix + ".values." + a_col_name
      try:
        value = row.get(col_name, "")
      except odb.eNoSuchColumn:
        value = ""
      if value is not None:
        hdf.setValue(nprefix, str(value))
      else:
        hdf.setValue(nprefix, "")

    ## generate key
    key = []
    for k in primaryKeys:
      val = row.get(k, "")
      if val:
        key.append("%s=%s" % (k, str(val)))
    hdf.setValue(prefix + ".key", string.join(key, "&"))

  def __findKeyFromQuery(self, hdf):
    fields = {}
    node = hdf.getObj("Query")
    if node:
      node = node.child()
      while node:
        if node.name() not in ("debug", ):
          fields[node.name()] = node.value()
        node = node.next()
    fields = fields.items()
    return fields
    
  def delete(self, hdf):
    fields = self.__findKeyFromQuery(hdf)
    tbl = self.db[self.tableName]
    row = tbl.fetchRow(fields)
    row.delete()
    
    self.redirectUri("list")

  def edit(self, hdf):
    hdf.setValue("CGI.cur.action", "edit")

    self.show(hdf)

    tbl = self.db[self.tableName]
    columnlist = tbl.getColumnList()
    self.__exportColumnHeaders(columnlist, hdf, "CGI.cur.table")
    hdf.setValue("CGI.cur.title", "Edit %s" % self.tableName)

  def Action_new(self, hdf):
    tbl = self.db[self.tableName]
    hdf.setValue("CGI.cur.action", "new")
    self.pagename = "edit"

    row = tbl.newRow()
    
    node = hdf.getObj("Query.values")
    if node:
      node = node.child()
      while node:
        row[node.name()] = node.value()
        node = node.next()

    row.save()
    self.redirectUri("list")

  def getKeyFromQuery(self, hdf):
    key = hdf.getValue("Query.key", "")
    if not key: return []
    parts = key.split("&")
    keylist = []
    for part in parts:
      kvparts = part.split("=", 1)
      keylist.append((kvparts[0], kvparts[1]))
    return keylist

  def Action_edit(self, hdf):
    hdf.setValue("CGI.cur.action", "edit")
    dict = {}
    node = hdf.getObj("Query.values")
    if node:
      node = node.child()
      while node:
        dict[node.name()] = node.value()
        node = node.next()
        
    tbl = self.db[self.tableName]

    keylist = self.getKeyFromQuery(hdf)
    row = tbl.fetchRow(keylist)

    for k,v in dict.items():
      row[k] = v
    row.save()
    self.redirectUri("list")

  def new(self, hdf):
    hdf.setValue("CGI.cur.title", "New %s Row" % self.tableName)

    hdf.setValue("CGI.cur.action", "new")
    self.pagename = "edit"

    tbl = self.db[self.tableName]
    prefix = "CGI.cur.row"
    columnlist = tbl.getColumnList()
    row = {}
    primaryKeys = tbl.getPrimaryKeyList()
    self.__exportRow(row, columnlist, hdf, "CGI.cur.row", primaryKeys)

    self.__exportColumnHeaders(columnlist, hdf, "CGI.cur.table")


  def show(self, hdf):
    hdf.setValue("CGI.cur.title", "Show %s" % self.tableName)

    fields = self.__findKeyFromQuery(hdf)

    tbl = self.db[self.tableName]
    row = tbl.fetchRow(fields)

    columnlist = tbl.getColumnList()
    primaryKeys = tbl.getPrimaryKeyList()
    self.__exportRow(row, columnlist, hdf, "CGI.cur.row", primaryKeys)
    

    
class DBScaffold(Scaffold):
  def setupDetails(self, db):
    self.db = db

  def setup(self, hdf):
    pathinfo = hdf.getValue("CGI.PathInfo", "")

    pathparts = []
    self.tableName = "__db"
    self.methodName = "listTables"

    if pathinfo: 
      pathparts = string.split(pathinfo, "/")

      self.methodName = "list"
      if len(pathparts) >= 1:
        self.tableName = pathparts[0]
      if len(pathparts) == 2:
        self.methodName = pathparts[1]
        if not self.methodName:
          self.methodName = "list"

  def display(self, hdf):

    if self.tableName == "__db":
      if self.methodName == "listTables":
        #tables = self.db.listTables()
        tables = self.db.getTableList()
        tables.sort()
        hdfhelp.hdfExportList("CGI.cur.tables", hdf, tables)
        Scaffold.display(self, hdf)
    else:
      s = Scaffold(self.context, self.methodName)
      s._path_num = self._path_num
      s.ncgi = self.ncgi

      self.pagename = self.methodName
      s.setupDetails(self.db, self.tableName)
      s.activate(hdf, self.methodName)
      self.pagename = s.pagename


  def list(self, hdf):
    hdf.setValue("CGI.cur.title", "List DB Tables")
    self.pagename = "tables"
    
    


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
