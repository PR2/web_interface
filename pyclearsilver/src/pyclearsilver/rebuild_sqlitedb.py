#! /usr/bin/env python

"""
usage: %(progname)s [db filenames...]
"""


import os, sys, string, time, getopt
from log import *

def dumpTable2(path, fn, tableName, fields):
  tfn = os.path.join(path, tableName + '.sql')
  warn("dumpTables", tfn)
  cmd = "sqlite3 %s '.schema %s' > %s" % (fn, tableName, tfn)
  warn(cmd)
  r = os.system(cmd)
  if r != 0:
    warn("return=", r)
    return False

  import sqlite3
  conn = sqlite3.connect(fn)
  c = conn.cursor()
  

  fieldList = []
  for field in fields:
    fieldList.append("%s > ''" % field)

  c.execute("SELECT %s FROM %s WHERE %s" % (string.join(fields, ","), tableName, string.join(fieldList, " AND ")))
  rows = c.fetchall()
  print len(rows)

  rows.sort()

  fp = open(tfn, "a")

  for row in rows:
    mnum = row[0]
    fieldList = []
    i = 0
    for field in fields:
      fieldList.append("%s='%s'" % (field, row[i]))
      i = i + 1

    open(".currentRow", "w").write("%s\n" % string.join(fieldList))
    try:
      c.execute("select * from %s where %s" % (tableName, string.join(fieldList, " AND ")))
    except:
      warn("Error with", row)
      continue
      
    row = c.fetchone()

    parts = []
    i = 0
    for col in row:
      if type(col) == type(None):
        parts.append("NULL")
      elif type(col) == type(1):
        parts.append(str(col))
      else:
        if c.rs.col_defs[i][0] == "odb_value": col = ""
        col = string.replace(col,"'","''")
        parts.append("'" + col + "'")
      i = i + 1
    sql = "INSERT INTO %s VALUES (%s);" % (tableName, string.join(parts, ", "))
    fp.write(sql + "\n")
    fp.flush()

    #    print mnum, len(rows)
  fp.close()
  return True
  

def dumpTable(path, fn, tableName):
  tfn = os.path.join(path, tableName + '.sql')
  warn("dumpTables", tfn)
  cmd = "sqlite3 %s '.dump %s' > %s" % (fn, tableName, tfn)
  warn(cmd)
  r = os.system(cmd)
  if r != 0:
    warn("return=", r)
    return False
  return True

def loadTable(path,fn, tableName):
  warn("loadTables", tableName)

  tfn = os.path.join(path, tableName + '.sql')

  cmd = "sqlite3 %s.new < %s" % (fn, tfn)
  warn(cmd)
  r = os.system(cmd)
  if r != 0:
    warn("return=", r)
    return False

  return True

def rebuildDatabase(fn):
  if not os.path.exists(fn): return

  path, f = os.path.split(fn)

  dumpPath = os.path.join(path, "dump")
  try: os.mkdir(dumpPath, 0700)
  except os.error, reason: pass

  cmd = "sqlite3 %s .tables" % fn
  fp = os.popen(cmd, "r")
  body = fp.read()
  fp.close()

  tables = body.split()

  #tables = ["mb_msgdata", "mb_msgthread"]
  #tables = ["mb_msgthread"]
  
  for tableName in tables:
    if tableName == "mb_msgdata":
      ret = dumpTable2(dumpPath, fn, tableName, fields=('mnum', ))
    elif tableName == "mb_thread":
      ret = dumpTable2(dumpPath, fn, tableName, fields=('thr_id', 'mnum'))
    else:
      ret = dumpTable(dumpPath, fn, tableName)
    if ret is False: return

  try: os.unlink(fn + ".new")
  except os.error, reason: pass

  for tableName in tables:
    ret = loadTable(dumpPath, fn, tableName)
    if ret is False: return
  
  now = int(time.time())
  dateStr = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
  warn(dateStr)

  os.rename(fn, fn + "_" + dateStr)
  os.rename(fn + ".new", fn)
    
  
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

  for fn in args:
    rebuildDatabase(fn)
  


if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
