#!/usr/bin/env python
#
# hdfhelp.py
#
# This code makes using odb with Clearsilver as "easy as stealing candy
# from a baby". - jeske
#
# How to use:
#
#  rows = tbl.fetchAllRows()
#  rows.hdfExport("CGI.rows", hdf_dataset)
#
#  row  = tbl.fetchRow( ('primary_key', value) )
#  row.hdfExport("CGI.row", hdf_dataset)
#
# How to setup:
#   
#  # define table
#  class AgentsTable(odb.Table):
#    def _defineRows(self):
#      self.d_addColumn("agent_id",kInteger,None,primarykey = 1,autoincrement = 1)
#      self.d_addColumn("login",kVarString,200,notnull=1)
#      self.d_addColumn("ticket_count",kIncInteger,None)
#
#    # make sure you return a subclass of hdfhelp.HdfRow
#
#    def defaultRowClass(self):
#      return hdfhelp.HdfRow
#    def defaultRowListClass(self):
#      return hdfhelp.HdfItemList
#

import string, os
import neo_cgi
import neo_cs
import neo_util
import odb
import time

import UserList

SECS_IN_MIN = 60
SECS_IN_HOUR = (SECS_IN_MIN * 60)
SECS_IN_DAY = (SECS_IN_HOUR * 24)
SECS_IN_WEEK = (SECS_IN_DAY * 7)
SECS_IN_MONTH = (SECS_IN_DAY * 30)

kYearPos = 0
kMonthPos = 1
kDayPos = 2
kHourPos = 3
kMinutePos = 4
kSecondPos = 5
kWeekdayPos = 6
kJulianDayPos = 7
kDSTPos = 8


def renderDate(then_time,day=0,date=1):
    if then_time is None:
        then_time = 0
    then_time = int(then_time)
    if then_time == 0 or then_time == -1:
        return ""
    
    then_tuple = time.localtime(then_time)

    now_tuple = time.localtime(time.time())
    
    if day or (then_tuple[kHourPos]==0 and then_tuple[kMinutePos]==0 and then_tuple[kSecondPos]==0):
        # it's just a date
        if then_tuple[kYearPos] == now_tuple[kYearPos]:
            # no year
            return time.strftime("%m/%d",then_tuple)
        else:
            # add year
            return time.strftime("%m/%d/%Y",then_tuple)
    elif date:
      return time.strftime("%m/%d/%Y",then_tuple)
    else:
        # it's a full time/date

        return time.strftime("%m/%d/%Y %I:%M%p",then_tuple)

class HdfRow(odb.Row):
    def hdfExport(self, prefix, hdf_dataset, *extra, **extranamed):
        skip_fields = extranamed.get("skip_fields", None)
        no_escape = extranamed.get("no_escape", None)
        translate_dict = extranamed.get("translate_dict", None)
        tz = extranamed.get("tz", "US/Pacific")

        for col_name,value in self.items():
            if skip_fields and (col_name in skip_fields):
                continue
            try:
                name,col_type,col_options = self._table.getColumnDef(col_name)
            except:
                col_type = odb.kVarString
                col_options = {}
            
	    if (value is not None):
                if col_options.get("no_export",0): continue
		if type(value) in [ type(0), type(0L) ]:
		    hdf_dataset.setValue(prefix + "." + col_name,"%d" % value)
                elif type(value) == type(1.0):
                    if int(value) == value:
                        hdf_dataset.setValue(prefix + "." + col_name,"%d" % value)
                    else:
                        hdf_dataset.setValue(prefix + "." + col_name,"%0.2f" % value)
		else:
                    if col_type == odb.kReal:
                        log("why are we here with this value: %s" % value)
                    if translate_dict:
                        for k,v in translate_dict.items():
                            value = string.replace(value,k,v)
                    if no_escape and col_name in no_escape:
                      hdf_dataset.setValue(prefix + "." + col_name,str(value))
                    else:
                      hdf_dataset.setValue(prefix + "." + col_name,neo_cgi.htmlEscape(str(value)))
                if col_options.get("int_date",0):
                    hdf_dataset.setValue(prefix + "." + col_name + ".string",renderDate(value))
                    hdf_dataset.setValue(prefix + "." + col_name + ".day_string",renderDate(value,day=1))
                    hdf_dataset.setValue(prefix + "." + col_name + ".date_string",renderDate(value,date=1))
                    if value: 
                      try:
                        neo_cgi.exportDate(hdf_dataset, "%s.%s" % (prefix, col_name), tz, value)
                      except TypeError:
                        pass

		if col_options.has_key("enum_values"):
		    enum = col_options["enum_values"]
		    hdf_dataset.setValue(prefix + "." + col_name + ".enum",
					 str(enum.get(value,'')))

class HdfItemList(UserList.UserList):
    def hdfExport(self,prefix,hdf_dataset,*extra,**extranamed):
        export_by = extranamed.get("export_by", None)
	n = 0
	for row in self:
            if export_by is not None:
                n = row[export_by]
	    row.hdfExport("%s.%d" % (prefix,n),hdf_dataset,*extra,**extranamed)
	    n = n + 1

def setList(hdf, prefix, lst):
    hdf.setValue(prefix+".0", str(len(lst)))
    for n in range(len(lst)):
        hdf.setValue(prefix+".%d" %(n+1), lst[n]);

def getList(hdf, name):
    lst = []
    for n in range(hdf.getIntValue(name,0)):
        lst.append(hdf.getValue(name+".%d" %(n+1), ""))

    return lst

def eval_cs(hdf,a_cs_string):
    cs = neo_cs.CS(hdf)
    try:
      cs.parseStr(a_cs_string)
      return cs.render()
    except:
      return "Error in CS tags: %s" % neo_cgi.htmlEscape(repr(a_cs_string))

def childloop(hdf):
    children = []
    if hdf:
        hdf = hdf.child()
        while hdf:
            children.append(hdf)
            hdf = hdf.next()
    return children

# ----------------------------

class HDF_Database(odb.Database):
  def defaultRowClass(self):
    return HdfRow
  def defaultRowListClass(self):
    return HdfItemList

# ----------------------------


def loopHDF(hdf, name=None):
  results = []
  if name: o = hdf.getObj(name)
  else: o = hdf
  if o:
    o = o.child()
    while o:
      results.append(o)
      o = o.next()
  return results


def loopKVHDF(hdf, name=None):
  results = []
  if name: o = hdf.getObj(name)
  else: o = hdf
  if o:
    o = o.child()
    while o:
      results.append((o.name(), o.value()))
      o = o.next()
  return results


class hdf_iterator:
  def __init__(self, hdf):
    self.hdf = hdf
    self.node = None
    if self.hdf:
      self.node = self.hdf.child()

  def __iter__(self): return self

  def next(self):
    if not self.node:
      raise StopIteration

    ret = self.node
    self.node = self.node.next()
      
    return ret

class hdf_kv_iterator(hdf_iterator):
  def next(self):
    if not self.node: raise StopIteration

    ret = (self.node.name(), self.node.value())
    self.node = self.node.next()
      
    return ret

class hdf_key_iterator(hdf_iterator):
  def next(self):
    if not self.node: raise StopIteration

    ret = self.node.name()
    self.node = self.node.next()
      
    return ret

class hdf_ko_iterator(hdf_iterator):
  def next(self):
    if not self.node: raise StopIteration

    ret = (self.node.name(), self.node)
    self.node = self.node.next()
      
    return ret
  

def hdfExportDict(prefix, hdf, dict):
  for k,v in dict.items():
    hdf.setValue(prefix + "." + str(k), str(v))


def hdfExportList(prefix, hdf, list):
  n = 0
  for item in list:
    n = n + 1
    hdf.setValue(prefix + "." + str(n), str(item))

def exportDate(hdf, prefix, tz, ttime):
  lt = time.localtime(ttime)
  hour = lt.tm_hour
  if hour >= 12: 
    am = 0; hour=hour-12
  else: am = 1
  hdf.setValue(prefix + ".hour", str(hour))
  hdf.setValue(prefix + ".sec", str(am))

  hdf.setValue(prefix + ".24hour", str(lt.tm_hour))
  hdf.setValue(prefix + ".min", "%02d" % lt.tm_min)
  hdf.setValue(prefix + ".sec", "%02d" % lt.tm_sec)

  hdf.setValue(prefix + ".mday", str(lt.tm_mday))
  hdf.setValue(prefix + ".mon", str(lt.tm_mon))
  hdf.setValue(prefix + ".year", str(lt.tm_year))
  hdf.setValue(prefix + ".2yr", str(lt.tm_year)[2:4])
  hdf.setValue(prefix + ".wday", str(lt.tm_wday))

  hdf.setValue(prefix + ".tzoffset", "0")


def exportGUID(prefix, hdf, guidStr):
  import guid
  try:
    t = guid.extract_time(guidStr)
    hdf.setValue(prefix + ".time_t", str(t))
    exportDate(hdf, prefix + ".time", "US/Pacific", t)
    hdf.setValue(prefix + ".random", str(guid.extract_random(guidStr)))
    hdf.setValue(prefix + ".server_id", str(guid.extract_ip(guidStr)))
  except guid.InvalidGUID:
    pass


def renderSize(prefix, hdf, size):
  s = ""
  units = ""
  if size > (1024*1024*1024):
    ret_size = "%.1f" % (size / (1024*1024*1024.))
    units = "G"
    s = "%s%s" % (ret_size, units)
  elif size > (1024*1024):
    ret_size = "%.1f" % (size / (1024*1024.))
    units = "M"
    s = "%s%s" % (ret_size, units)
  elif size > (1024):
    ret_size = "%.1f" % (size / (1024.))
    units = "K"
  else:
    ret_size = str(size)
  if ret_size.endswith(".0"): ret_size = ret_size[:-2]
  s = "%s%s" % (ret_size, units)
  hdf.setValue(prefix + ".str", s)

# ----------------------------

def test():
    import neo_util
    hdf = neo_util.HDF()
    hdf.setValue("foo","1")
    print eval_cs(hdf,"this should say 1  ===> <?cs var:foo ?>")
    

if __name__ == "__main__":
    test()






