#!/usr/bin/env python
#
# odb.py
#
# Object Database Api
#
# Written by David Jeske <jeske@chat.net>, 2001/07. 
# Inspired by eGroups' sqldb.py originally written by Scott Hassan circa 1998.
#
# Copyright (C) 2001, by David Jeske
#
# Goals:
#       - a simple object-like interface to database data
#       - database independent (someday)
#       - relational-style "rigid schema definition"
#       - object style easy-access
#
# Example:
#
#  import odb
#
#  # define table
#  class AgentsTable(odb.Table):
#    def _defineRows(self):
#      self.d_addColumn("agent_id",kInteger,None,primarykey = 1,autoincrement = 1)
#      self.d_addColumn("login",kVarString,200,notnull=1)
#      self.d_addColumn("ticket_count",kIncInteger,None)
#
#  if __name__ == "__main__":
#
#    # open database
#    import odb_mysql
#    conn = odb_mysql.Connection(host = 'localhost',
#                               user='username', 
#                               passwd = 'password', 
#                               db='testdb')
#    db = Database(conn)
#    tbl = AgentsTable(db,"agents")
#
#    # create row
#    agent_row = tbl.newRow()
#    agent_row.login = "foo"
#    agent_row.save()
#
#    # fetch row (must use primary key)
#    try:
#      get_row = tbl.fetchRow( ('agent_id', agent_row.agent_id) )
#    except odb.eNoMatchingRows:
#      print "this is bad, we should have found the row"
#
#    # fetch rows (can return empty list)
#    list_rows = tbl.fetchRows( ('login', "foo") )
#

import os, sys, string, types, re
import zlib
import marshal

from log import *
if 0:
  debugfull()
  LOGGING_STATUS[DEV_UPDATE] = 1
  LOGGING_STATUS[DEV_SELECT] = 1
  LOGGING_STATUS[DEV_REPORT] = 1
else:
  debugoff()


import weakref

import handle_error

class odb_Exception(Exception):
  def __init__(self, message):
    self.message = message

  def __str__(self):
    return repr(self.message)

class eNoSuchColumn(odb_Exception):
  pass

class eNonUniqueMatchSpec(odb_Exception):
  pass

class eNoMatchingRows(odb_Exception):
  pass
class eInternalError(odb_Exception):
  pass
class eInvalidMatchSpec(odb_Exception):
  pass
class eInvalidData(odb_Exception):
  pass
class eUnsavedObjectLost(odb_Exception):
  pass
class eDuplicateKey(odb_Exception):
  pass
class eInvalidJoinSpec(odb_Exception):
  pass

#####################################
# COLUMN TYPES                       
################                     ######################
# typename     ####################### size data means:
#              #                     # 
## kInteger       = "kInteger"          # -
## kFixedString   = "kFixedString"      # size
## kVarString     = "kVarString"        # maxsize
## kBigString     = "kBigString"        # -
## kIncInteger    = "kIncInteger"       # -
## kDateTime      = "kDateTime"
## kTimeStamp     = "kTimeStamp"
## kReal          = "kReal"

## kEnumeration   = "kEnumeration"          # -


DEBUG = 0

class _ODB_Object:
  def get(self, data, options):
    return data

  def set(self, val, options):
    return val

  def convertTo(self, data, options):  
    try:
      return str(data)
    except UnicodeEncodeError:
      return data.encode("utf-8")

  def convertFrom(self, val, options):  
    return val

  def needQuoting(self): return False
  def needEscape(self): return False
  def needEncode(self): return False
  def compressionOk(self): return False

class _ODB_Integer(_ODB_Object):
  def odbType(self): return "kInteger"
  def sqlColType(self, options): 
    return "integer"

  ## convertTo - converts 'data' to database representation from the 
  ## local representation
  def convertTo(self, data, options):
    try:
      return str(data)
    except (ValueError,TypeError):
      raise eInvalidData, data

  ## convertFrom - converts 'val' from database representation to the 
  ## local representation
  def convertFrom(self, val, options):
    try:
      return int(val)
    except ValueError:
      return val

  def needEscape(self): return False

class _ODB_IncInteger(_ODB_Integer):
  def odbType(self): return "kIncInteger"
  def sqlColType(self, options):
    return "integer"

class _ODB_Enumeration(_ODB_Integer):
  def odbType(self): return "kEnumeration"
  def set(self, data, options):
    try:
      n = options["enum_values"][data]
    except KeyError:
      raise eInvalidData, data
    return n
    
  def get(self, val, options):  
    return options['inv_enum_values'][int(val)]


class _ODB_FixedString(_ODB_Object):
  def odbType(self): return "kFixedString"
  def sqlColType(self, options):
    sz = options.get('size', None)
    if sz is None: coltype = 'char'
    else:  coltype = "char(%s)" % sz

    return coltype

  def needEscape(self): return True
  def needQuoting(self): return True

class _ODB_VarString(_ODB_FixedString):
  def odbType(self): return "kVarString"
  def sqlColType(self, options):
    sz = options.get('size', None)
    if sz is None: coltype = 'varchar'
    else:  coltype = "varchar(%s)" % sz
    return coltype

class _ODB_BigString(_ODB_FixedString):
  def odbType(self): return "kBigString"
  def sqlColType(self, options): return "text"

  def convertTo(self, data, options):
    if options.get("compress_ok", False):
      cdata = zlib.compress(data, 9)
      if len(cdata) < len(data):
        return cdata
    return data

  def convertFrom(self, val, options):
    if options.get('compress_ok', False) and val:
      try:
        data = zlib.decompress(val)
      except zlib.error:
        data = val
      return data
    return val

  def needEscape(self): return True
  def compressionOk(self): return True

class _ODB_Blob(_ODB_BigString):
  def odbType(self): return "kBlob"
  def sqlColType(self, options): return "text"

  def needEscape(self): return False
  def needEncode(self): return True
  def compressionOk(self): return True

class _ODB_DateTime(_ODB_FixedString):
  def odbType(self): return "kDateTime"
  def sqlColType(self, options): return "datetime"

class _ODB_TimeStamp(_ODB_FixedString):
  def odbType(self): return "kTimeStamp"
  def sqlColType(self, options): return "timestamp"

class _ODB_CreatedStamp(_ODB_FixedString):
  def sqlColType(self, options): return "integer"
  def odbType(self): return "kCreatedStamp"
  def beforeInsert(self, row, colname):
    row[colname] = int(time.time())

class _ODB_CreatedStampMS(_ODB_FixedString):
  def sqlColType(self, options): return "real"
  def odbType(self): return "kCreatedStampMS"
  def beforeInsert(self, row, colname):
    row[colname] = time.time()

class _ODB_ModifiedStamp(_ODB_CreatedStamp):
  def odbType(self): return "kModifiedStamp"
  def beforeUpdate(self, row, colname):
    row[colname] = int(time.time())
    
    


class _ODB_Real(_ODB_Object):
  def odbType(self): return "kReal"
  def sqlColType(self, options): return "real"

  def convertTo(self, val, options):
    return str(val)

  def convertFrom(self, val, options):  
    try:
      return float(val)
    except (ValueError,TypeError):
      raise eInvalidData, val

  def needEscape(self): return False

import guid
class _ODB_GUID(_ODB_FixedString):
  def odbType(self): return "kGUID"
  def sqlColType(self, options):
    return "char(40)"
  def generate(self):
    return guid.generate()

####
import fixedpoint

class ODB_FixedPoint(_ODB_VarString):
  def convertTo(self, data, options):  
    return str(data)

  def convertFrom(self, val, options):  
    try:
      return fixedpoint.FixedPoint(val, 2)
    except TypeError:
      return val

kFixedPoint = ODB_FixedPoint()
####
  

kInteger = _ODB_Integer()
kIncInteger = _ODB_IncInteger()
kFixedString = _ODB_FixedString()
kVarString = _ODB_VarString()
kBigString = _ODB_BigString()
kBlob = _ODB_Blob()
kDateTime = _ODB_DateTime()
kTimeStamp = _ODB_TimeStamp()

kModifiedStamp = _ODB_ModifiedStamp()
kCreatedStamp = _ODB_CreatedStamp()
kCreatedStampMS = _ODB_CreatedStampMS()

kReal = _ODB_Real()
kEnumeration = _ODB_Enumeration()
kGUID = _ODB_GUID()

def parseFieldType(dataStr):
  patStr = "([a-z]+)(\(([0-9]+)\))?"
  pat = re.compile(patStr)
  dataStr = dataStr.lower().strip()
  m = pat.match(dataStr)
  if not m:
    raise TypeError

  dataType = m.group(1)
  arg = m.group(3)
  
  if dataType == "integer":
    fieldType = kInteger
  elif dataType == "varchar":
    fieldType = kVarString
  elif dataType == "real":
    fieldType = kReal
  elif dataType == "datetime":
    fieldType = kDateTime
  elif dataType == "timestamp":
    fieldType = kTimeStamp
  elif dataType == "text":
    fieldType = kBigString
  else:
    fieldType = kVarString

  return fieldType

class Cursor:
  def __init__(self, cursor):
    self.cursor = cursor

  def description(self): return self.cursor.description
  def arraysize(self): return self.cursor.arraysize
  def rowcount(self): return self.cursor.rowcount

  def execute(self, sql):
    try:
      return self.cursor.execute(sql)
    except:
      warn(sql)
      raise

  def fetchone(self):
    return self.cursor.fetchone()

  def fetchmany(self, size=None, keep=None):
    return self.cursor.fetchmany(size=size, keep=keep)

  def fetchall(self):
    return self.cursor.fetchall()

  def insert_id(self):
    raise "Unimplemented Error"

  def close(self):
    return self.cursor.close()

class Connection:
  def __init__(self):
    self._conn = None

  def cursor(self):
    return Cursor(self._conn.cursor())

  def begin(self):
    pass

  def commit(self):
    return self._conn.commit()

  def rollback(self):
    return self._conn.rollback()

  def close(self):
    return self._conn.close()

  def auto_increment(self, coltype):
    return coltype, "AUTO_INCREMENT"

  def createTable(self, sql, cursor):
    return sql

  def supportsTriggers(self): return False

  def listTriggers(self): 
    raise Unimplemented, "triggers are not implemented in this connection type."


##############
# DATABASE
#
# this will ultimately turn into a mostly abstract base class for
# the DB adaptors for different database types....
#

class Database:
    def __init__(self, conn, debug=0):
        self._tables = {}
        self.conn = conn
        self._cursor = None
        self.compression_enabled = False
        self.debug = debug
        self.SQLError = conn.SQLError

        self.__defaultRowClass = self.defaultRowClass()
        self.__defaultRowListClass = self.defaultRowListClass()

    def getTableList(self):
      tblList = []
      for tblName in self._tables.keys():
        if tblName.find("_repl_") == 0: continue
        tblList.append(tblName)
      return tblList

    def hasReplication(self):
      if self._tables.has_key("_repl_log"): return True
      return False

    def enabledCompression(self):
      self.compression_enabled = True

    def defaultCursor(self):
        if self._cursor is None:
            self._cursor = self.conn.cursor()
        return self._cursor

    def escape_string(self, str):
        def subfn(m):
            c = m.group(0)
            return "%%%02X" % ord(c)

        return re.sub("('|\0|%)",subfn,str)

    def unescape_string(self, str):
        def subfn(m):
            hexnum = int(m.group(1),16)
            return "%c" % hexnum
        return re.sub("%(..)",subfn,str)


    def escape(self,str):
      return self.conn.escape(str)
    def encode(self,str):
      return self.conn.encode(str)
    def decode(self,str):
      return self.conn.decode(str)

    def getDefaultRowClass(self): return self.__defaultRowClass
    def setDefaultRowClass(self, clss): self.__defaultRowClass = clss
    def getDefaultRowListClass(self): return self.__defaultRowListClass
    def setDefaultRowListClass(self, clss): self.__defaultRowListClass = clss

    def defaultRowClass(self):
        return Row

    def defaultRowListClass(self):
        # base type is list...
        return list

    def addTable(self, attrname, tblname, tblclass, 
                 rowClass = None, 
                 check = 0, 
                 create = 0, 
                 rowListClass = None,
                 replication = None):
        tbl = tblclass(self, tblname, rowClass=rowClass, check=check, 
                       create=create, rowListClass=rowListClass,
                       replication=replication)
        self._tables[attrname] = tbl
        return tbl

    def close(self):
##         for name, tbl in self._tables.items():
##             tbl.db = None
        self._tables = {}

        if self.conn is not None:
          cursor = self.defaultCursor()
          cursor.close()
          self._cursor = None

          self.conn.commit()
          self.conn.close()
          self.conn = None

    def __del__(self):
      self.close()

    def __getitem__(self, tblname):
      if not self._tables:
        raise AttributeError, "odb.Database: not initialized properly, self._tables does not exist"

      try:
        return self._tables[tblname]
      except KeyError:
        raise AttributeError, "odb.Database: unknown table %s" % (tblname)


    def __getattr__(self, key):
        if key == "_tables":
            raise AttributeError, "odb.Database: not initialized properly, self._tables does not exist"

        try:
            table_dict = getattr(self,"_tables")
            return table_dict[key]
        except KeyError:
            raise AttributeError, "odb.Database: unknown attribute %s" % (key)
        
    def beginTransaction(self, cursor=None):
        if cursor is None:
            cursor = self.defaultCursor()
        dlog(DEV_UPDATE,"begin")
        self.conn.begin()
        #cursor.execute("begin")

    def commitTransaction(self, cursor=None):
        if cursor is None:
            cursor = self.defaultCursor()
        dlog(DEV_UPDATE,"commit")
        self.conn.commit()
        #cursor.execute("commit")

    def rollbackTransaction(self, cursor=None):
        if cursor is None:
            cursor = self.defaultCursor()
        dlog(DEV_UPDATE,"rollback")
        self.conn.rollback()
        #cursor.execute("rollback")

    ## 
    ## schema creation code
    ##

    def createTables(self):
      tables = self.listTables()

      for attrname, tbl in self._tables.items():
        tblname = tbl.getTableName()

        if tblname not in tables:
#          warn("table %s does not exist" % tblname)
          tbl.createTable()
        else:
          invalidAppCols, invalidDBCols = tbl.checkTable()

##          self.alterTableToMatch(tbl)

    def createIndices(self):
      for attrname, tbl in self._tables.items():
        indices = self.listIndices(tbl.getTableName())
        for indexName, (columns, unique) in tbl.getIndices().items():
          if indexName in indices: continue

#          warn("creating index for %s for %s" % (tbl.getTableName(), str(columns)))
          tbl.createIndex(columns, indexName=indexName, unique=unique)

    def dropIndices(self):
      cursor = self.defaultCursor()
      indices = self.listIndices("")
      for indexName in indices:
        sql = "DROP INDEX %s" % indexName
        cursor.execute(sql)

    def createTriggers(self):
      triggers = self.listTriggers()

      for attrname, tbl in self._tables.items():
        for triggerName, triggerSQL in tbl._triggers.items():

          if triggerName in triggers:
            self.dropTrigger(triggerName)
            triggers.remove(triggerName)
          self.createTrigger(triggerName, triggerSQL)

      if triggers:
        for trigger in triggers:
          self.dropTrigger(triggerName)

    def createTrigger(self, triggerName, sql, cursor=None):
      if cursor is None: cursor = self.defaultCursor()
      cursor.execute(sql)

    def dropTrigger(self, triggerName, cursor=None):
      if cursor is None: cursor = self.defaultCursor()
      sql = "DROP TRIGGER %s" % triggerName
      cursor.execute(sql)

    ## parse the schema of an existing db and build table objects to
    ## reflect the schema.
    def reflect(self):
      tables = self.listTables()
      for tablename in tables:
        tbl = self.addTable(tablename, tablename, _ReflectTable)
      

    def synchronizeSchema(self):
      tables = self.listTables()

      cursor = self.defaultCursor()
      for attrname, tbl in self._tables.items():
        tblname = tbl.getTableName()
        self.conn.alterTableToMatch(tbl, cursor)

      self.createIndices()
      if self.conn.supportsTriggers():
        self.createTriggers()
        
    def listTables(self, cursor=None):
      if cursor is None: cursor = self.defaultCursor()
      return self.conn.listTables(cursor)

    def listTriggers(self, cursor=None):
      if cursor is None: cursor = self.defaultCursor()
      return self.conn.listTriggers(cursor)
    
    def listIndices(self, tableName, cursor=None):
      if cursor is None: cursor = self.defaultCursor()
      return self.conn.listIndices(tableName, cursor)
      

    def listFieldsDict(self, table_name, cursor=None):
      if cursor is None: cursor = self.defaultCursor()
      return self.conn.listFieldsDict(table_name, cursor)

    def listFields(self, table_name, cursor=None):
      columns = self.listFieldsDict(table_name, cursor=cursor)
      return columns.keys()


##########################################
# Table
#


class Table:
    def subclassinit(self):
        pass
    def __init__(self,database,table_name,
                 rowClass = None, 
                 check = 0, 
                 create = 0, 
                 rowListClass = None,
                 replication = None):
        self.__db = weakref.ref(database)
        self.__table_name = table_name
        self.__replication = replication

        if rowClass:
            self.__defaultRowClass = rowClass
        else:
            self.__defaultRowClass = database.getDefaultRowClass()

        if rowListClass:
            self.__defaultRowListClass = rowListClass
        else:
            self.__defaultRowListClass = database.getDefaultRowListClass()

        # get this stuff ready!
        
        self.__column_list = []
        self.__vcolumn_list = []
        self.__columns_locked = 0
        self.__has_value_column = 0

        self.__indices = {}
        self._triggers = {}

        # this will be used during init...
        self.__col_def_hash = None
        self.__vcol_def_hash = None
        self.__primary_key_list = None
        self.__relations_by_table = {}

        self.__fullTextSearchable = False

        # ask the subclass to def his rows
        self._defineRows()

        if self.__replication:
          self.__replication.addTable(self)

        # get ready to run!
        self.__lockColumnsAndInit()

        self._defineRelations()

        self.subclassinit()
        
        if create:
            self.createTable()

        if check:
            self.checkTable()

    def hasReplication(self):
      if self.__replication is None: return False
      return True

    def getReplication(self):
      return self.__replication

    def _colTypeToSQLType(self, colname, coltype, options, singlePrimaryKey=0):
      coltype = coltype.sqlColType(options)

      coldef = ""

      if options.get('notnull', 0): coldef = coldef + " NOT NULL"
      if options.get('autoincrement', 0): 
        coltype, acoldef = self.getDB().conn.auto_increment(coltype)
        if acoldef:
          coldef = coldef + " " + acoldef

      if options.get('unique', 0): coldef = coldef + " UNIQUE"
      
      if singlePrimaryKey:
        if options.get('primarykey', 0): coldef = coldef + " PRIMARY KEY"

      if options.has_key('default'):
        defaultValue = options.get('default')
        if defaultValue is None:
          coldef = coldef + " DEFAULT NULL"
        elif type(defaultValue) in (types.IntType, types.LongType, types.FloatType):
          coldef = coldef + " DEFAULT %s" % defaultValue
        else:
          coldef = coldef + " DEFAULT '%s'" % defaultValue
        

      coldef = "%s %s %s" % (colname, coltype, coldef)

      return coldef

    def getDB(self):
      return self.__db()

    def getTableName(self):  return self.__table_name
    def setTableName(self, tablename):  self.__table_name = tablename

    def getIndices(self): return self.__indices

    def _createTableSQL(self):
      primarykeys = self.getPrimaryKeyList()
      singlePrimaryKey = 0
      if len(primarykeys) == 1:  singlePrimaryKey = 1
        
      defs = []
      for colname, coltype, options in self.__column_list:
        defs.append(self._colTypeToSQLType(colname, coltype, options, singlePrimaryKey))

      defs = string.join(defs, ", ")

      primarykey_str = ""
      if singlePrimaryKey == 0:
        primarykeys = self.getPrimaryKeyList()
        if primarykeys:
          primarykey_str = ", PRIMARY KEY (" + string.join(primarykeys, ",") + ")"

      if self.__fullTextSearchable:
        sql = self.getDB().conn.create_fullTextSearchTable(self.__table_name, self.__column_list)
      else:
        sql = "CREATE TABLE %s (%s %s)" % (self.__table_name, defs, primarykey_str)
      return sql

    def createTable(self, cursor=None):
      if cursor is None: cursor = self.__db().defaultCursor()
      sql = self._createTableSQL()

      sql = self.__db().conn.createTable(sql, cursor)

      debug("CREATING TABLE:", sql)

      cursor.execute(sql)

    def dropTable(self, cursor=None):
      if cursor is None: cursor = self.__db().defaultCursor()
      try:
        cursor.execute("drop table %s" % self.__table_name)   # clean out the table
      except self.getDB().SQLError, reason:
        pass

    def deleteAllRows(self, cursor=None):
      if cursor is None: cursor = self.__db().defaultCursor()
      try:
        cursor.execute("delete from %s" % self.__table_name)   # clean out the table
      except self.getDB().SQLError, reason:
        pass

    def renameTable(self, newTableName, cursor=None):
      if cursor is None: cursor = self.__db().defaultCursor()
      try:
        cursor.execute("rename table %s to %s" % (self.__table_name, newTableName))
      except self.getDB().SQLError, reason:
        pass

      self.setTableName(newTableName)
      
    def getTableColumnsFromDB(self):
      return self.__db().listFieldsDict(self.__table_name)
      
    def checkTable(self, warnflag=1):
      invalidDBCols = {}
      invalidAppCols = {}

      dbcolumns = self.getTableColumnsFromDB()
      for coldef in self.__column_list:
        colname = coldef[0]

        dbcoldef = dbcolumns.get(colname, None)
        if dbcoldef is None:
          invalidAppCols[colname] = 1
      
      for colname, row in dbcolumns.items():
        coldef = self.__col_def_hash.get(colname, None)
        if coldef is None:
          invalidDBCols[colname] = 1

      if self.__fullTextSearchable:
        if 'docid' in invalidAppCols:  del invalidAppCols['docid']
        if 'rowid' in invalidAppCols:  del invalidAppCols['rowid']

      if warnflag == 1:
        if invalidDBCols:
          warn("----- WARNING ------------------------------------------")
          warn("  There are columns defined in the database schema that do")
          warn("  not match the application's schema: %s" % self.getTableName())
          warn("  columns:", invalidDBCols.keys())
          warn("--------------------------------------------------------")

        if invalidAppCols: 
          warn("----- WARNING ------------------------------------------")
          warn("  There are new columns defined in the application schema")
          warn("  that do not match the database's schema: %s" % self.getTableName())
          warn("  columns:", invalidAppCols.keys())
          warn("--------------------------------------------------------")

      return invalidAppCols, invalidDBCols


    def alterTableToMatch(self, cursor=None):
      if cursor is None: cursor = self.defaultCursor()
      return self.conn.alterTableToMatch(cursor)

    def addIndex(self, columns, indexName=None, unique=0):
      if indexName is None:
        indexName = self.getTableName() + "_index_" + string.join(columns, "_")

      self.__indices[indexName] = (columns, unique)
      
    def createIndex(self, columns, indexName=None, unique=0, cursor=None):
      if cursor is None: cursor = self.__db().defaultCursor()
      cols = string.join(columns, ",")

      if indexName is None:
        indexName = self.getTableName() + "_index_" + string.join(columns, "_")

      uniquesql = ""
      if unique:
        uniquesql = " UNIQUE"
      sql = "CREATE %s INDEX %s ON %s (%s)" % (uniquesql, indexName, self.getTableName(), cols)
      debug("creating index: ", sql)
      cursor.execute(sql)


    ## Column Definition

    def hasColumn(self, column_name):
      try:
        coldef = self.getColumnDef(column_name)
      except eNoSuchColumn:
        return False
      return True

    def getColumnDef(self,column_name):
        try:
            return self.__col_def_hash[column_name]
        except KeyError:
            try:
                return self.__vcol_def_hash[column_name]
            except KeyError:
              ## handle joined columns
              if column_name.startswith("_"):
                parts = column_name[1:].split(".")
                if len(parts) == 2:
                  table_column_name = parts[0]
                  column_name = parts[1]

                  c_name,c_type,c_options = self.__col_def_hash[table_column_name]
                  foreign_table = c_options["foreign_table"]
                  foreign_key = c_options["foreign_key"]

                  a_table = self.getDB()[foreign_table]
                  return a_table.getColumnDef(column_name)

              raise eNoSuchColumn("no column (%s) on table '%s'" % (column_name,self.__table_name))

    def getColumnList(self):  
      return self.__column_list + self.__vcolumn_list
    def getAppColumnList(self): 
      return self.__column_list

    def databaseSizeForData_ColumnName_(self,data,col_name):
        try:
            col_def = self.__col_def_hash[col_name]
        except KeyError:
            try:
                col_def = self.__vcol_def_hash[col_name]
            except KeyError:
                raise eNoSuchColumn("no column (%s) on table %s" % (col_name,self.__table_name))

        c_name,c_type,c_options = col_def

        if c_type == kBigString:
            if c_options.get("compress_ok",0) and self.__db().compression_enabled:
                z_size = len(zlib.compress(data,9))
                r_size = len(data)
                if z_size < r_size:
                    return z_size
                else:
                    return r_size
            else:
                return len(data)
        else:
            # really simplistic database size computation:
            try:
                a = data[0]
                return len(data)
            except:
                return 4

    def getColumnOption(self, columnName, optionName):
      a,b,options = self.getColumnDef(columnName)
      return options[optionName]
      

    def columnType(self, col_name):
        try:
            col_def = self.__col_def_hash[col_name]
        except KeyError:
            try:
                col_def = self.__vcol_def_hash[col_name]
            except KeyError:
                raise eNoSuchColumn("no column (%s) on table %s" % (col_name,self.__table_name))

        c_name,c_type,c_options = col_def
        return c_type

    def convertDataForColumn(self,data,col_name):
        try:
            col_def = self.__col_def_hash[col_name]
        except KeyError:
            try:
                col_def = self.__vcol_def_hash[col_name]
            except KeyError:
                raise eNoSuchColumn("no column (%s) on table %s" % (col_name,self.__table_name))

        c_name,c_type,c_options = col_def

        if c_type == kIncInteger:
            raise eInvalidData("invalid operation for column (%s:%s) on table (%s)" % (col_name,c_type,self.__table_name))

        if data is None: return None

        try:
          val = c_type.set(data, c_options)
          return val
        except eInvalidData, reason:
          raise eInvalidData("invalid data (%s) for col (%s:%s) on table (%s)" % (repr(data),col_name,c_type,self.__table_name))


    def getPrimaryKeyList(self):
      if self.__primary_key_list is not None:
        return tuple(self.__primary_key_list)
      
      primary_keys = []
      for col_name, ctype, options in self.__column_list:
        if options.get('primarykey', 0): primary_keys.append(col_name)

      return tuple(primary_keys)
    
    def hasValueColumn(self):
        return self.__has_value_column

    def hasColumn(self,name):
        return self.__col_def_hash.has_key(name)
    def hasVColumn(self,name):
        return self.__vcol_def_hash.has_key(name)
        

    def _defineRows(self):
        raise odb_Exception("can't instantiate base odb.Table type, make a subclass and override _defineRows()")

    def _defineRelations(self):
        pass

    def __lockColumnsAndInit(self):
        # add a 'odb_value column' before we lockdown the table def
        if self.__has_value_column:
            self.d_addColumn("odb_value",kBlob,None, default='', notnull=1)
#            self.d_addColumn("odb_value",kBigString,None, default='', notnull=1)

        self.__columns_locked = 1
        # walk column list and make lookup hashes, primary_key_list, etc..

        primary_key_list = []
        col_def_hash = {}
        for a_col in self.__column_list:
            name,type,options = a_col
            col_def_hash[name] = a_col
            if options.has_key('primarykey'):
                primary_key_list.append(name)

        self.__col_def_hash = col_def_hash
        self.__primary_key_list = primary_key_list

        # setup the value columns!

        if (not self.__has_value_column) and (len(self.__vcolumn_list) > 0):
            raise odb_Exception("can't define vcolumns on table without ValueColumn, call d_addValueColumn() in your _defineRows()")

        vcol_def_hash = {}
        for a_col in self.__vcolumn_list:
            name,type,options = a_col
            vcol_def_hash[name] = a_col

        self.__vcol_def_hash = vcol_def_hash
        
        
    def __checkColumnLock(self):
        if self.__columns_locked:
            raise odb_Exception("can't change column definitions outside of subclass' _defineRows() method!")

    # table definition methods, these are only available while inside the
    # subclass's _defineRows method
    #
    # Ex:
    #
    # import odb
    # class MyTable(odb.Table):
    #   def _defineRows(self):
    #     self.d_addColumn("id",kInteger,primarykey = 1,autoincrement = 1)
    #     self.d_addColumn("name",kVarString,120)
    #     self.d_addColumn("type",kInteger,
    #                      enum_values = { 0 : "alive", 1 : "dead" }

    def d_addColumn(self,col_name,ctype,size=None,primarykey = 0, 
                    notnull = 0,indexed=0,
                    default=None,
                    unique=0,
                    autoincrement=0,
                    autoguid=0,
                    safeupdate=0,
                    enum_values = None,
                    no_export = 0,
                    relations=None,
                    foreign_key=None,
                    compress_ok=0,
                    int_date=0):

        self.__checkColumnLock()
        if ctype in (kCreatedStamp, kModifiedStamp):
          int_date = 1

        options = {}
        options['default']       = default
        if primarykey:
            options['primarykey']    = primarykey
        if unique:
            options['unique']        = unique
        if indexed:
            options['indexed']       = indexed
            self.addIndex((col_name,))
        if safeupdate:
            options['safeupdate']    = safeupdate
        if autoincrement:
            options['autoincrement'] = autoincrement
        if autoguid:
            options['autoguid'] = autoguid
            if ctype != kGUID:
              raise eInvalidData("cannot set autoguid for non-kGUID columns")
        if notnull:
            options['notnull']       = notnull
        if size:
            options['size']          = size
        if no_export:
            options['no_export']     = no_export
        if int_date:
            if ctype not in (kInteger, kCreatedStamp, kModifiedStamp):
                raise eInvalidData("can't flag columns int_date unless they are kInteger")
            else:
                options['int_date'] = int_date
            
        if enum_values:
            options['enum_values']   = enum_values
            inv_enum_values = {}
            for k,v in enum_values.items():
                if inv_enum_values.has_key(v):
                    raise eInvalidData("enum_values parameter must be a 1 to 1 mapping for Table(%s)" % self.__table_name)
                else:
                    inv_enum_values[v] = k
            options['inv_enum_values'] = inv_enum_values
        if foreign_key:
            try:
              foreign_table, foreign_column_name = foreign_key.split(".")
            except ValueError:
              foreign_table = foreign_key
              foreign_column_name = col_name
            options['foreign_table']      = foreign_table
            options['foreign_key']      = foreign_column_name
            
            self.__relations_by_table[foreign_table] = (col_name, foreign_column_name)
        if relations:
            options['relations']      = relations
            for a_relation in relations:
                table, foreign_column_name = a_relation
                if self.__relations_by_table.has_key(table):
                    raise eInvalidData("multiple relations for the same foreign table are not yet supported" )
                self.__relations_by_table[table] = (col_name,foreign_column_name)
        if compress_ok and self.__db().compression_enabled:
          if ctype.compressionOk():
            options['compress_ok'] = 1
          else:
            raise eInvalidData("this column cannot be compress_ok=1")
        
        self.__column_list.append( (col_name,ctype,options) )

    def d_addInsertTrigger(self, triggerName, tsql):
      sql = "CREATE TRIGGER %s INSERT ON %s\n  BEGIN\n  %s;\n  END;" % (triggerName, self.getTableName(), tsql)
      self._triggers[triggerName] = sql

    def d_addUpdateTrigger(self, triggerName, tsql):
      sql = "CREATE TRIGGER %s UPDATE ON %s\n  BEGIN\n  %s;\n  END;" % (triggerName, self.getTableName(), tsql)
      self._triggers[triggerName] = sql

    def d_addUpdateColumnsTrigger(self, triggerName, columns, tsql):
      sql = "CREATE TRIGGER %s UPDATE OF %s ON %s\n  BEGIN\n  %s;\n  END;" % (triggerName, string.join(columns, ","), self.getTableName(), tsql)
      self._triggers[triggerName] = sql

    def d_addDeleteTrigger(self, triggerName, tsql):
      sql = "CREATE TRIGGER %s DELETE ON %s\n  BEGIN\n  %s;\n  END;" % (triggerName, self.getTableName(), tsql)
      self._triggers[triggerName] = sql


    def d_addValueColumn(self):
        self.__checkColumnLock()
        self.__has_value_column = 1

    def d_addVColumn(self,col_name,type,size=None,default=None):
        self.__checkColumnLock()

        if (not self.__has_value_column):
            raise odb_Exception("can't define VColumns on table without ValueColumn, call d_addValueColumn() first")

        options = {}
        if default:
            options['default'] = default
        if size:
            options['size']    = size

        self.__vcolumn_list.append( (col_name,type,options) )

    def getRelations(self):
      return self.__relations_by_table

    def d_fullTextSearch(self):
      self.__fullTextSearchable = True

    def d_belongsTo(self, col_name, tblNameStr=None, foreign_key=None, order=None):
      if foreign_key is None: foreign_key = col_name

      self.__relations_by_table[tblNameStr] = (col_name, foreign_key)

    def d_hasMany(self, tblname, col_name, foreign_key=None, order=None):
      if foreign_key is None: foreign_key = col_name
      self.__relations_by_table[tblname] = (col_name, foreign_key)

    def d_hasOne(self, col_name, tblname, foreign_key=None, order=None):
      if foreign_key is None: foreign_key = col_name

      a,b,options = self.getColumnDef(col_name)
      options['foreign.table'] = tblname
      options['foreign.key'] = foreign_key
      self.__relations_by_table[tblname] = (col_name, foreign_key)
      

    #####################
    # _checkColMatchSpec(col_match_spec,should_match_unique_row = 0)
    #
    # raise an error if the col_match_spec contains invalid columns, or
    # (in the case of should_match_unique_row) if it does not fully specify
    # a unique row.
    #
    # NOTE: we don't currently support where clauses with value column fields!
    #
    
    def _fixColMatchSpec(self,col_match_spec, should_match_unique_row = 0):
        if type(col_match_spec) == type([]):
            if type(col_match_spec[0]) != type((0,)):
                raise eInvalidMatchSpec("invalid types in match spec, use [(,)..] or (,)")
        elif type(col_match_spec) == type((0,)):
            col_match_spec = [ col_match_spec ]
        elif type(col_match_spec) == type(None):
            if should_match_unique_row:
                raise eNonUniqueMatchSpec("can't use a non-unique match spec (%s) here" % col_match_spec)
            else:
                return None
        else:
            raise eInvalidMatchSpec("invalid types in match spec, use [(,)..] or (,)")

        unique_column_lists = []

        if should_match_unique_row:

            # first the primary key list
            my_primary_key_list = []
            for a_key in self.__primary_key_list:
                my_primary_key_list.append(a_key)

            # then other unique keys
            for a_col in self.__column_list:
                col_name,a_type,options = a_col
                if options.has_key('unique'):
                    unique_column_lists.append( (col_name, [col_name]) )

            for indexName, (columns, unique) in self.getIndices().items():
              if unique:
                unique_column_lists.append((indexName, list(columns)))

            unique_column_lists.append( ('primary_key', my_primary_key_list) )
                
        new_col_match_spec = []
        for a_col in col_match_spec:
            name,val = a_col
            # newname = string.lower(name)
            #  what is this doing?? - jeske
            newname = name
            if not self.__col_def_hash.has_key(newname):
                raise eNoSuchColumn("no such column in match spec: '%s'" % str(newname))

            new_col_match_spec.append( (newname,val) )

            if should_match_unique_row:
                for name,a_list in unique_column_lists:
                    try:
                        a_list.remove(newname)
                    except ValueError:
                        # it's okay if they specify too many columns!
                        pass

        if should_match_unique_row:
            for name,a_list in unique_column_lists:
                if len(a_list) == 0:
                    # we matched at least one unique colum spec!
                    # log("using unique column (%s) for query %s" % (name,col_match_spec))
                    return new_col_match_spec


              
            
            raise eNonUniqueMatchSpec("can't use a non-unique match spec (%s) here" % col_match_spec)

        return new_col_match_spec

    def __buildWhereClause (self, col_match_spec,other_clauses = None):
        sql_where_list = []

        if not col_match_spec is None:
            for m_col in col_match_spec:
                m_col_name,m_col_val = m_col
                c_name,c_type,c_options = self.__col_def_hash[m_col_name]

                c_name = "%s.%s" % (self.getTableName(), c_name)

                if m_col_val is None:
                  sql_where_list.append("%s = NULl" % (c_name,))
                else:
                  try:
                    val = c_type.convertFrom(m_col_val, c_options)
                  except eInvalidData, data:
                    raise eInvalidData("invalid literal for %s in table %s" % (repr(m_col_val),self.__table_name))


                  if c_type.needEscape():
                    val2 = self.__db().escape(val)
                  elif c_type.needEncode():
                    val2 = self.__db().encode(val)
                  else:
                    val2 = val

                  if c_type.needQuoting():
                    sql_where_list.append("%s = '%s'" % (c_name, val2))
                  else:
                    sql_where_list.append("%s = %s" % (c_name, val2))


        if other_clauses is None:
            pass
        elif type(other_clauses) == type(""):
            sql_where_list = sql_where_list + [other_clauses]
        elif type(other_clauses) == type([]):
            sql_where_list = sql_where_list + other_clauses
        else:
            raise eInvalidData("unknown type of extra where clause: %s" % repr(other_clauses))
                    
        return sql_where_list

    def __fetchRows(self,col_match_spec,cursor = None, where = None, 
                    order_by = None, limit_to = None,
                    skip_to = None, join = None,
                    join2 = None,
                    column_list = None,
                    raw_rows = False):
        if cursor is None:
            cursor = self.__db().defaultCursor()

        # build column list
        sql_columns = []
        if column_list is None:
          column_list = map(lambda x: x[0], self.__column_list)

        for name in column_list:
          sql_columns.append("%s.%s" % (self.__table_name, name))

        # build join information

        joined_cols = []
        joined_cols_hash = {}
        join_clauses = []
        if not join is None:
            for a_table,retrieve_foreign_cols in join:
                try:
                  if isinstance(a_table, Table):
                    atbl = a_table
                    a_table = atbl.getTableName()
                  else:
                    parts = a_table.split(".")
                    atbl = self
                    for atbln in parts[:-1]:
                      atbl = self.getDB()[atbln]
                    a_table = parts[-1]
                        
                  my_col,foreign_col = self.__relations_by_table[a_table]
                except KeyError,reason:
                  raise eInvalidJoinSpec("can't find table %s in defined relations for %s (%s)  reason=%s" % (a_table,self.__table_name, repr(self.__relations_by_table.items()), reason))

                for a_col in retrieve_foreign_cols:
                    full_col_name = "%s.%s" % (a_table,a_col)
                    joined_cols_hash[full_col_name] = 1
                    joined_cols.append(full_col_name)
                    sql_columns.append( full_col_name )

                join_clauses.append(" left join %s on %s.%s=%s.%s " % (a_table,atbl.getTableName(),my_col,a_table, foreign_col))

        if not join2 is None:
          for col in join2:
            c_name,c_type,c_options = self.__col_def_hash[col]
            foreign_table = c_options["foreign_table"]
            foreign_key = c_options["foreign_key"]
            
            
            a_table = self.getDB()[foreign_table]

            #joinTable = "_%s_%s" % (col, a_table.getTableName(), )
            joinTable = "_%s" % (col, )
            joinColumn = "%s.%s" % (joinTable, foreign_key)
            
            for col_name, ctype, options in a_table.getAppColumnList():
              full_col_name = "%s.%s" % (joinTable, col_name)

              joined_cols_hash[full_col_name] = 1
              joined_cols.append(full_col_name)
              sql_columns.append(full_col_name)

            join_clauses.append(" left join %s AS %s on %s.%s=%s " % (a_table.getTableName(), joinTable, self.getTableName(), col, joinColumn))

        
        # start buildling SQL
        sql = "SELECT %s FROM %s" % (string.join(sql_columns,","),
                                     self.__table_name)
        
        # add join clause
        if join_clauses:
            sql = sql + string.join(join_clauses," ")
        
        # add where clause elements
        sql_where_list = self.__buildWhereClause (col_match_spec,where)
        if sql_where_list:
            sql = sql + " WHERE %s" % (string.join(sql_where_list," and "))

        # add order by clause
        if order_by:
          ob = []
          for col in order_by:
            order = "asc"
            if type(col) == types.TupleType:
              col,order = col
            elif type(col) == types.StringType:
              aparts = col.split(" ", 1)
              if len(aparts) == 2:
                col,order = aparts

            if col.find(".") == -1:
              obstr = "%s.%s" % (self.__table_name, col)
            else:
              obstr = col

            if order == "desc":
              obstr = obstr + " " + order

            ob.append(obstr)
              
              
          sql = sql + " ORDER BY %s " % string.join(ob,",")

        # add limit
        if not limit_to is None:
            if not skip_to is None:
##                log("limit,skip = %s,%s" % (limit_to,skip_to))
              if self.__db().conn.getConnType() == "sqlite":
                sql = sql + " LIMIT %s OFFSET %s " % (limit_to,skip_to)
              else:
                sql = sql + " LIMIT %s, %s" % (skip_to,limit_to)
            else:
                sql = sql + " LIMIT %s" % limit_to
        else:
            if not skip_to is None:
                raise eInvalidData("can't specify skip_to without limit_to in MySQL")

        dlog(DEV_SELECT,sql)

        #warn(sql)
        try:
          cursor.execute(sql)
        except:
          warn(sql)
          raise

        # create defaultRowListClass instance...
        return_rows = self.__defaultRowListClass()
            
        # should do fetchmany!
        all_rows = cursor.fetchall()
        if raw_rows == True:  ## bug out is the user justs want the raw rows
          return all_rows

        for a_row in all_rows:
            data_dict = {}

            col_num = 0
            
            #            for a_col in cursor.description:
            #                (name,type_code,display_size,internal_size,precision,scale,null_ok) = a_col
            for fullname in sql_columns:
                parts = string.split(fullname, ".", 1)
                table = parts[0]
                name = parts[1]

                if self.__col_def_hash.has_key(name) or joined_cols_hash.has_key(fullname):
                    # only include declared columns!
                    if joined_cols_hash.has_key(fullname):
                      data_dict[fullname] = a_row[col_num]
                    elif self.__col_def_hash.has_key(name):
                      c_name,c_type,c_options = self.__col_def_hash[name]
                      if a_row[col_num] is None:
                        data_dict[name] = None
                      else:
                        aval = a_row[col_num]

                        if c_type.needEncode():
                          aval = self.__db().decode(aval)
                        data_dict[name] = c_type.convertFrom(aval, c_options)
                    else:
                      data_dict[name] = a_row[col_num]
                        
                    col_num = col_num + 1

            newrowobj = self.__defaultRowClass(self,data_dict,joined_cols = joined_cols)
            return_rows.append(newrowobj)
              

            
        return return_rows

    def __deleteRow(self,a_row,cursor = None):
        if cursor is None:
            cursor = self.__db().defaultCursor()

        # build the where clause!
        match_spec = a_row.getPKMatchSpec()
        sql_where_list = self.__buildWhereClause (match_spec)

        sql = "DELETE FROM %s WHERE %s" % (self.__table_name,
                                           string.join(sql_where_list," and "))
        dlog(DEV_UPDATE,sql)
        cursor.execute(sql)

        if self.__replication:
          self.__replication.deleteRow(self, a_row)

       

    def __updateRowList(self,a_row_list,cursor = None):
        if cursor is None:
            cursor = self.__db().defaultCursor()

        for a_row in a_row_list:
            for name,c_type,options in self.__column_list:
              if hasattr(c_type, "beforeUpdate"):
                c_type.beforeUpdate(a_row, name)

            update_list = a_row.changedList()

            # build the set list!
            sql_set_list = []

            for a_change in update_list:
                col_name,col_val,col_inc_val = a_change
                c_name,c_type,c_options = self.__col_def_hash[col_name]

                if c_type != kIncInteger and col_val is None:
                    sql_set_list.append("%s = NULL" % c_name)
                elif c_type == kIncInteger and col_inc_val is None:
                    sql_set_list.append("%s = 0" % c_name)
                else:
                  if c_type == kIncInteger:
                    sql_set_list.append("%s = %s + %d" % (c_name,c_name,long(col_inc_val)))                    
                  else:
                    if col_val is None:
                      sql_set_list.append("%s = NULL" % c_name)
                    else:
                      val = c_type.convertTo(col_val, c_options)

                      if c_type.needEscape():
                        val2 = self.__db().escape(val)
                      elif c_type.needEncode():
                        val2 = self.__db().encode(val)
                      else:
                        val2 = val

                      if c_type.needQuoting():
                        sql_set_list.append("%s = '%s'" % (c_name, val2))
                      else:
                        sql_set_list.append("%s = %s" % (c_name, val2))


            # build the where clause!
            match_spec = a_row.getPKMatchSpec()
            sql_where_list = self.__buildWhereClause (match_spec)

            if sql_set_list:
                sql = "UPDATE %s SET %s WHERE %s" % (self.__table_name,
                                                 string.join(sql_set_list,","),
                                                 string.join(sql_where_list," and "))

                dlog(DEV_UPDATE,sql)
                try:
                    cursor.execute(sql)
                except Exception, reason:
                    if string.find(str(reason), "Duplicate entry") != -1:
                        raise eDuplicateKey(reason)
                    raise odb_Exception(reason)

                if self.__replication:
                  self.__replication.updateRow(self, a_row)

                a_row.markClean()

    def __insertRow(self,a_row_obj,cursor = None,replace=0):
        if cursor is None:
            cursor = self.__db().defaultCursor()

        sql_col_list = []
        sql_data_list = []
        auto_increment_column_name = None

        a_row_obj.changedList()

        for name,c_type,options in self.__column_list:
            try:
              if not a_row_obj.has_key(name):
                if hasattr(c_type, "beforeInsert"):
                  c_type.beforeInsert(a_row_obj, name)
              
              data = a_row_obj._getRaw(name, convert=0)

              sql_col_list.append(name)
              if data is None:
                sql_data_list.append("NULL")
              else:
                if c_type.needEscape():
                  val = c_type.convertTo(data, options)
                  val2 = self.__db().escape(val)
                elif c_type.needEncode():
                  val = c_type.convertTo(data, options)
                  val2 = self.__db().encode(val)
                else:
                  val2 = data

                if c_type.needQuoting():
                  sql_data_list.append("'%s'" % val2)
                else:
                  sql_data_list.append(str(val2))

            except KeyError, reason:
              if options.has_key("autoguid"):
                sql_col_list.append(name)
                a_row_obj[name] = c_type.generate()
                sql_data_list.append("'%s'" % a_row_obj[name])
              elif options.has_key("autoincrement"):
                if auto_increment_column_name:
                  raise eInternalError("two autoincrement columns (%s,%s) in table (%s)" % (auto_increment_column_name, name,self.__table_name))
                else:
                  auto_increment_column_name = name

        if replace:
            sql = "REPLACE INTO %s (%s) VALUES (%s)" % (self.__table_name,
                                                   string.join(sql_col_list,","),
                                                   string.join(sql_data_list,","))
        else:
          sql = "INSERT INTO %s (%s) VALUES (%s)" % (self.__table_name,
                                                   string.join(sql_col_list,","),
                                                   string.join(sql_data_list,","))

        dlog(DEV_UPDATE,sql)

        try:
          cursor.execute(sql)
        except Exception, reason:
          # sys.stderr.write("errror in statement: " + sql + "\n")
          log("error in statement: " + sql + "\n")
          if string.find(str(reason), "Duplicate entry") != -1:
            raise eDuplicateKey(reason)
          raise odb_Exception(reason)

        if self.__replication:
          self.__replication.updateRow(self, a_row_obj)
            
        if auto_increment_column_name:
          a_row_obj[auto_increment_column_name] = cursor.insert_id(self.__table_name, auto_increment_column_name)

    # ----------------------------------------------------
    #   Helper methods for Rows...
    # ----------------------------------------------------


        
    #####################
    # r_deleteRow(a_row_obj,cursor = None)
    #
    # normally this is called from within the Row "delete()" method
    # but you can call it yourself if you want
    #

    def r_deleteRow(self,a_row_obj, cursor = None):
        curs = cursor
        self.__deleteRow(a_row_obj, cursor = curs)


    #####################
    # r_updateRow(a_row_obj,cursor = None)
    #
    # normally this is called from within the Row "save()" method
    # but you can call it yourself if you want
    #

    def r_updateRow(self,a_row_obj, cursor = None):
        curs = cursor
        self.__updateRowList([a_row_obj], cursor = curs)

    #####################
    # InsertRow(a_row_obj,cursor = None)
    #
    # normally this is called from within the Row "save()" method
    # but you can call it yourself if you want
    #

    def r_insertRow(self,a_row_obj, cursor = None,replace=0):
        curs = cursor
        self.__insertRow(a_row_obj, cursor = curs,replace=replace)


    # ----------------------------------------------------
    #   Public Methods
    # ----------------------------------------------------


        
    #####################
    # deleteRow(col_match_spec)
    #
    # The col_match_spec paramaters must include all primary key columns.
    #
    # Ex:
    #    a_row = tbl.fetchRow( ("order_id", 1) )
    #    a_row = tbl.fetchRow( [ ("order_id", 1), ("enterTime", now) ] )


    def deleteRow(self,col_match_spec, where=None):
        n_match_spec = self._fixColMatchSpec(col_match_spec)
        cursor = self.__db().defaultCursor()

        # build sql where clause elements
        sql_where_list = self.__buildWhereClause (n_match_spec,where)
        if not sql_where_list:
            return

        sql = "DELETE FROM %s WHERE %s" % (self.__table_name, string.join(sql_where_list," and "))

        dlog(DEV_UPDATE,sql)
        cursor.execute(sql)
        
    #####################
    # fetchRow(col_match_spec)
    #
    # The col_match_spec paramaters must include all primary key columns.
    #
    # Ex:
    #    a_row = tbl.fetchRow( ("order_id", 1) )
    #    a_row = tbl.fetchRow( [ ("order_id", 1), ("enterTime", now) ] )


    def fetchRow(self, col_match_spec, cursor = None, join2=None):
        n_match_spec = self._fixColMatchSpec(col_match_spec, should_match_unique_row = 1)

        rows = self.__fetchRows(n_match_spec, cursor = cursor, join2=join2)
        if len(rows) == 0:
            raise eNoMatchingRows("no row matches %s" % repr(n_match_spec))

        if len(rows) > 1:
            raise eInternalError("unique where clause shouldn't return > 1 row")

        return rows[0]
            

    #####################
    # fetchRows(col_match_spec)
    #
    # Ex:
    #    a_row_list = tbl.fetchRows( ("order_id", 1) )
    #    a_row_list = tbl.fetchRows( [ ("order_id", 1), ("enterTime", now) ] )


    def fetchRows(self, col_match_spec = None, cursor = None, 
                  where = None, order_by = None, limit_to = None, 
                  skip_to = None, join = None,
                  join2 = None,
                  column_list = None,
                  raw_rows = False):
        n_match_spec = self._fixColMatchSpec(col_match_spec)

        return self.__fetchRows(n_match_spec,
                                cursor = cursor,
                                where = where,
                                order_by = order_by,
                                limit_to = limit_to,
                                skip_to = skip_to,
                                join = join,
                                join2 = join2,
                                column_list = column_list,
                                raw_rows = raw_rows)

    def fetchRowCount (self, col_match_spec = None, 
                       cursor = None, where = None):
        n_match_spec = self._fixColMatchSpec(col_match_spec)
        sql_where_list = self.__buildWhereClause (n_match_spec,where)
        sql = "SELECT COUNT(*) FROM %s" % self.__table_name
        if sql_where_list:
            sql = "%s WHERE %s" % (sql,string.join(sql_where_list," and "))
        if cursor is None:
          cursor = self.__db().defaultCursor()
        dlog(DEV_SELECT,sql)
        cursor.execute(sql)
        try:
            count, = cursor.fetchone()
        except TypeError:
            count = 0
        return count


    #####################
    # fetchAllRows()
    #
    # Ex:
    #    a_row_list = tbl.fetchRows( ("order_id", 1) )
    #    a_row_list = tbl.fetchRows( [ ("order_id", 1), ("enterTime", now) ] )

    def fetchAllRows(self, join2=None):
        try:
            return self.__fetchRows([], join2=join2)
        except eNoMatchingRows:
            # else return empty list...
            return self.__defaultRowListClass()

    def newRow(self,replace=0,save=0,**kws):
        row = self.__defaultRowClass(self,None,create=1,replace=replace)
        for (cname, ctype, opts) in self.__column_list:
            if opts['default'] is not None and ctype is not kIncInteger:
                row[cname] = opts['default']
        if kws:
          for k,v in kws.items():
           row[k] = v

        if save:
          row.save()
                
        return row

    def fetchRowUsingPrimaryKey(self, *args):
      kl = self.getPrimaryKeyList()

      if len(kl) != len(args):
        raise eInternalData("wrong number of primary key arguments")

      keylist = []
      i = 0
      for field in kl:
        keylist.append((field, args[i]))
        i = i + 1
                        
      return self.fetchRow(keylist)

    def lookup(self, join2=None, **kws):
      keylist = []
      for k,v in kws.items():
        keylist.append((k,v))
        
      try:
        row = self.fetchRow(keylist, join2=join2)
      except eNoMatchingRows:
        row = None
      return row

    def lookupRows(self, join2=None, **kws):
      keylist = []
      for k,v in kws.items():
        keylist.append((k,v))
        
      try:
        rows = self.fetchRows(keylist, join2=join2)
      except eNoMatchingRows:
        rows = []
      return rows
      
    def lookupCreate(self, **kws):
      row = self.lookup(**kws)

      if row is None:
        row = self.newRow()
        for k,v in kws.items():
          row[k] = v

      return row
        
        
class Row:
    __instance_data_locked  = 0
    def subclassinit(self):
        pass

    def __init__(self,_table,data_dict,create=0,joined_cols = None,replace=0):

        self._inside_getattr = 0  # stop recursive __getattr__
        self._table = _table
        self._should_insert = create or replace
        self._should_replace = replace
        self._rowInactive = None
        self._joinedRows = []
        
        self.__pk_match_spec = None
        self.__vcoldata = {}
        self.__inc_coldata = {}

        self.__joined_cols_dict = {}
        for a_col in joined_cols or []:
            self.__joined_cols_dict[a_col] = 1
        
        if create:
            self.__coldata = {}
        else:
            if type(data_dict) != type({}):
                raise eInternalError, "rowdict instantiate with bad data_dict"
            self.__coldata = data_dict
            self.__unpackVColumn()

        self.markClean()

        self.subclassinit()
        self.__instance_data_locked = 1

    def getTable(self): 
      return self._table

    def getDB(self):
      return self._table.getDB()

    def joinRowData(self,another_row):
        self._joinedRows.append(another_row)

    def getPKMatchSpec(self):
        return self.__pk_match_spec

    def isClean(self):
      changed_list = self.changedList()
      if len(changed_list):
        return 0
      return 1

    def markClean(self):
        self.__vcolchanged = 0
        self.__colchanged_dict = {}

        for key in self.__inc_coldata.keys():
            self.__coldata[key] = self.__coldata.get(key, 0) + self.__inc_coldata[key]

        self.__inc_coldata = {}

        if not self._should_insert:
            # rebuild primary column match spec
            new_match_spec = []
            for col_name in self._table.getPrimaryKeyList():
                try:
                    rdata = self[col_name]
                except KeyError:
                    raise eInternalError, "must have primary key data filled in to save %s:Row(col:%s)" % (self._table.getTableName(),col_name)
                    
                new_match_spec.append( (col_name, rdata) )
            self.__pk_match_spec = new_match_spec

    def __unpackVColumn(self):
        if self._table.hasValueColumn():
          if self.__coldata.has_key("odb_value") and self.__coldata['odb_value']:
            val = self.__coldata['odb_value']
            val2 = self.getDB().unescape_string(val)

            try:
              self.__vcoldata = marshal.loads(val2)
            except ValueError:
##               #warn(self)
##               val2 = self.getDB().decode(val)
##               warn("val2", repr(val2))
##               self.__vcoldata = marshal.loads(val2)
              raise
        
    def __packVColumn(self):
        if self._table.hasValueColumn():
          self.__coldata['odb_value'] = self.getDB().escape_string(marshal.dumps(self.__vcoldata))
          self.__colchanged_dict['odb_value'] = 1
          

    ## ----- utility stuff ----------------------------------

    def __del__(self):
        # check for unsaved changes
        changed_list = self.changedList()
        if len(changed_list):
            info = "unsaved Row for table (%s) lost, call discard() to avoid this error. Lost changes: %s\n" % (self._table.getTableName(), repr(changed_list)[:256])
            if 0:
                raise eUnsavedObjectLost, info
            else:
                sys.stderr.write(info)
                

    def __repr__(self):
        return "Row from (%s): %s" % (self._table.getTableName(),repr(self.__coldata) + repr(self.__vcoldata))

    ## ---- class emulation --------------------------------

    def __getattr__(self,key):
        if self._inside_getattr:
          raise AttributeError, "recursively called __getattr__ (%s,%s)" % (key,self._table.getTableName())
        try:
            self._inside_getattr = 1
            try:
                return self[key]
            except KeyError:
                if self._table.hasColumn(key) or self._table.hasVColumn(key):
                    return None
                else:
                    raise AttributeError, "unknown field '%s' in Row(%s)" % (key,self._table.getTableName())
        finally:
            self._inside_getattr = 0

    def __setattr__(self,key,val):
        if not self.__instance_data_locked:
            self.__dict__[key] = val
        else:
            my_dict = self.__dict__
            if my_dict.has_key(key):
                my_dict[key] = val
            else:
                # try and put it into the rowdata
                try:
                    self[key] = val
                except KeyError, reason:
                    raise AttributeError, reason


    ## ---- dict emulation ---------------------------------

    def _getRaw(self, key, convert=1):
      self.checkRowActive()
      
      try:
          c_name, c_type, c_options = self._table.getColumnDef(key)
      except eNoSuchColumn:
          # Ugh, this sucks, we can't determine the type for a joined
          # row, so we just default to kVarString and let the code below
          # determine if this is a joined column or not
          c_type = kVarString
          c_options = {}
          c_name = key

      if c_type == kIncInteger:
          c_data = self.__coldata.get(key, 0) 
          if c_data is None: c_data = 0
          i_data = self.__inc_coldata.get(key, 0)
          if i_data is None: i_data = 0
          return c_data + i_data

      try:
        if convert:
          return c_type.get(self.__coldata[key], c_options)
        else:
          return self.__coldata[key]

      except KeyError:
          try:
              return self.__vcoldata[key]
          except KeyError:
              for a_joined_row in self._joinedRows:
                  try:
                      return a_joined_row[key]
                  except KeyError:
                      pass

              raise KeyError, "unknown column %s in '%s'" % (key,self.getTable().getTableName())
      
    
    def __getitem__(self,key):
      return self._getRaw(key)

    def __setitem__(self,key,data):
        self.checkRowActive()
        
        try:
            newdata = self._table.convertDataForColumn(data,key)
        except eNoSuchColumn, reason:
            raise KeyError, reason

        if self._table.hasColumn(key):
            self.__coldata[key] = newdata
            self.__colchanged_dict[key] = 1
        elif self._table.hasVColumn(key):
            self.__vcoldata[key] = newdata
            self.__vcolchanged = 1
        else:
            for a_joined_row in self._joinedRows:
                try:
                    a_joined_row[key] = data
                    return
                except KeyError:
                    pass
            raise KeyError, "unknown column name %s" % key


    def __delitem__(self,key,data):
        self.checkRowActive()
        
        if self.table.hasVColumn(key):
            del self.__vcoldata[key]
        else:
            for a_joined_row in self._joinedRows:
                try:
                    del a_joined_row[key]
                    return
                except KeyError:
                    pass
            raise KeyError, "unknown column name %s" % key


    def copyFrom(self,source):
        for name,t,options in self._table.getColumnList():
            if not options.has_key("autoincrement"):
                self[name] = source[name]


    # make sure that .keys(), and .items() come out in a nice order!

    def keys(self):
        self.checkRowActive()
        
        key_list = []
        for name,t,options in self._table.getColumnList():
            key_list.append(name)
        for name in self.__joined_cols_dict.keys():
            key_list.append(name)

        for a_joined_row in self._joinedRows:
            key_list = key_list + a_joined_row.keys()
            
        return key_list


    def items(self):
        self.checkRowActive()
        
        item_list = []
        for name,t,options in self._table.getColumnList():
            item_list.append( (name,self[name]) )

        for name in self.__joined_cols_dict.keys():
            item_list.append( (name,self[name]) )

        for a_joined_row in self._joinedRows:
            item_list = item_list + a_joined_row.items()


        return item_list

    def values(elf):
        self.checkRowActive()

        value_list = self.__coldata.values() + self.__vcoldata.values()

        for a_joined_row in self._joinedRows:
            value_list = value_list + a_joined_row.values()

        return value_list
        

    def __len__(self):
        self.checkRowActive()
        
        my_len = len(self.__coldata) + len(self.__vcoldata)

        for a_joined_row in self._joinedRows:
            my_len = my_len + len(a_joined_row)

        return my_len

    def has_key(self,key):
        self.checkRowActive()
        
        if self.__coldata.has_key(key) or self.__vcoldata.has_key(key):
            return 1
        else:

            for a_joined_row in self._joinedRows:
                if a_joined_row.has_key(key):
                    return 1
            return 0
        
    def get(self,key,default = None):
        self.checkRowActive()

        
        
        if self.__coldata.has_key(key):
            return self.__coldata[key]
        elif self.__vcoldata.has_key(key):
            return self.__vcoldata[key]
        else:
            for a_joined_row in self._joinedRows:
                try:
                    return a_joined_row.get(key,default)
                except eNoSuchColumn:
                    pass

            if self._table.hasColumn(key):
                return default
            
            raise eNoSuchColumn, "no such column %s" % key

    def inc(self,key,count=1):
        self.checkRowActive()

        if self._table.hasColumn(key):
            try:
                self.__inc_coldata[key] = self.__inc_coldata[key] + count
            except KeyError:
                self.__inc_coldata[key] = count

            self.__colchanged_dict[key] = 1
        else:
            raise AttributeError, "unknown field '%s' in Row(%s)" % (key,self._table.getTableName())
    

    ## ----------------------------------
    ## real interface


    def fillDefaults(self):
        for field_def in self._table.fieldList():
            name,type,size,options = field_def
            if options.has_key("default"):
                self[name] = options["default"]

    ###############
    # changedList()
    #
    # returns a list of tuples for the columns which have changed
    #
    #   changedList() -> [ ('name', 'fred'), ('age', 20) ]

    def changedList(self):
        if self.__vcolchanged:
          self.__packVColumn()

        changed_list = []
        for a_col in self.__colchanged_dict.keys():
            changed_list.append( (a_col,self.get(a_col,None),self.__inc_coldata.get(a_col,None)) )

        return changed_list

    def discard(self):
        self.__coldata = None
        self.__vcoldata = None
        self.__colchanged_dict = {}
        self.__vcolchanged = 0

    def delete(self,cursor = None):
        self.checkRowActive()
        
        fromTable = self._table
        curs = cursor
        fromTable.r_deleteRow(self,cursor=curs)
        self._rowInactive = "deleted"



    def save(self,cursor = None):
        toTable = self._table

        self.checkRowActive()

        if self._should_insert:
            toTable.r_insertRow(self,replace=self._should_replace)
            self._should_insert = 0
            self._should_replace = 0
            self.markClean()  # rebuild the primary key list
        else:
            curs = cursor
            toTable.r_updateRow(self,cursor = curs)

        # the table will mark us clean!
        # self.markClean()

    def checkRowActive(self):
        if self._rowInactive:
            raise eInvalidData, "row is inactive: %s" % self._rowInactive

    def databaseSizeForColumn(self,key):
        return self._table.databaseSizeForData_ColumnName_(self[key],key)

    

## ----------------------------------------------------------------------

class _ReflectTable(Table):
  def _defineRows(self):
    fields = self.getDB().listFieldsDict(self.getTableName())
    for fieldname, dict in fields.items():
      fieldStr = dict[2]

      fieldType = parseFieldType(fieldStr)

      self.d_addColumn(fieldname, fieldType)


## ----------------------------------------------------------------------

class ReplicationLog:
  def __init__(self, db):
    self.__db = weakref.ref(db)

    self._server_guid = None

    self.getDB().addTable("_repl_keyval", "repl_keyval", Replication_KeyValueTable)
    self.getDB().addTable("_repl_log", "repl_log", Replication_LogTable, 
                          rowClass = Replication_LogRow)
    self.getDB().addTable("_repl_deleted", "repl_deleted", Replication_DeletedTable)

  def getDB(self):
    return self.__db()

  def addTable(self, tbl):
    tbl.d_addColumn("__modified", kModifiedStamp, no_export=1)

  def getServerGUID(self):
    if not self._server_guid:
      row = self.getDB()._repl_keyval.lookup(key="server_guid")
      self._server_guid = row.val

    return self._server_guid

  def __getPrimaryKeyForTable(self, tbl, row):
    keyList = []

    for col_name in tbl.getPrimaryKeyList():
      val = str(row[col_name])
      keyList.append("%s,%s,%s" % (col_name, len(val), val))
    key = string.join(keyList, ",")
    return key

  def __recordUpdate(self, tbl, key):
    rrow = self.getDB()._repl_log.newRow(replace=1)
    rrow.tableName = tbl.getTableName()
    rrow.server_guid = self.getServerGUID()
    rrow.key = key
    rrow.save()

  def updateRow(self, tbl, row):
    ##warn("updateRow", tbl.getTableName(), whereList, changeSet)

    key = self.__getPrimaryKeyForTable(tbl, row)
    self.__recordUpdate(tbl, key)

  def deleteRow(self, tbl, row):
    #warn("deleteRow", tbl.getTableName(), row)

    key = self.__getPrimaryKeyForTable(tbl, row)
    #warn("key", key)

    drow = self.getDB()._repl_deleted.newRow(replace=1)
    drow.tableName = tbl.getTableName()
    drow.key = key
    drow.save()

    self.__recordUpdate(tbl, key)

  def getLogSince(self, tableName, startTime, endTime):
    rows = self.getDB()._repl_log.fetchRows(('tableName', tableName), where = ['timestamp >= %s' % startTime, 'timestamp <= %s' % endTime])
    return rows

    
    
    

class Replication_KeyValueTable(Table):
  def _defineRows(self):
    self.d_addColumn("key", kVarString, primarykey = 1)
    self.d_addColumn("val", kVarString)

  def createTable(self, cursor=None):
    Table.createTable(self, cursor=cursor)

    self.__makeServerGUID()

  def __makeServerGUID(self):
    server_guid = guid.generate()
    row = self.getDB()._repl_keyval.newRow(replace=1,key="server_guid", val=server_guid)
    row.save()
    return server_guid
    

class Replication_LogTable(Table):
  def _defineRows(self):
    self.d_addColumn("server_guid", kGUID, primarykey = 1)
    # the server guid who changed this key

    self.d_addColumn("timestamp", kCreatedStampMS, primarykey = 1)
    # when the change took place.

    self.d_addColumn("rep_guid", kGUID, primarykey = 1, autoguid=1)

    self.d_addColumn("tableName", kVarString)

    self.d_addColumn("key", kVarString)
    # the primarykey of the change
    # [columnName,length,data]...

class Replication_LogRow(Row):
  def pkey(self):
    return repl_parsePrimaryKey(self.key)
    

class Replication_DeletedTable(Table):
  def _defineRows(self):
    self.d_addColumn("tableName", kVarString, primarykey = 1)
    # the table where the key was deleted

    self.d_addColumn("key", kVarString, primarykey = 1)  
    # the deleted primarykey 

    self.d_addColumn("timestamp", kCreatedStampMS)
    # timestamp of the deletion

def repl_parsePrimaryKey(key):
  i = 0

  keyList = []
  while 1:
    j = key.find(",", i)
    if j == -1: break
    columnName = key[i:j]

    j = j + 1
    k = key.find(",", j)
    if k == -1: break
    
    valLength = key[j:k]

    k = k + 1
    i = k + int(valLength)

    val = key[k:i]
    
    keyList.append((columnName, val))
  return keyList
    
    
    
