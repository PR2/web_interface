#!/usr/bin/env python

"""
usage: %(progname)s [args]
"""

# GUID.py
# Version 2.1. 
#
# Copyright (C) 2003  Dr. Conan C. Albrecht <conan_albrechtATbyu.edu>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA



##################################################################################################
###   A globally-unique identifier made up of time and ip and 8 random digits: 16 characters wide
###  
###   A globally unique identifier that combines ip, time, and random bits.  Since the 
###   time is listed first, you can sort records by guid.  You can also extract the time 
###   and ip if needed.  
###     
###   GUIDs make wonderful database keys.  They require no access to the 
###   database (to get the max index number), they are extremely unique, and they sort 
###   automatically by time.   GUIDs prevent key clashes when merging
###   two databases together, combining data, or generating keys in distributed
###   systems.
###
###   There is an Internet Draft for UUIDs, but this module does not implement it.
###   If the draft catches on, perhaps I'll conform the module to it.
###


# Changelog
# Sometime, 1997     Created the Java version of GUID
#                    Went through many versions in Java
# Sometime, 2002     Created the Python version of GUID, mirroring the Java version
# November 24, 2003  Changed Python version to be more pythonic, took out object and made just a module
# December 2, 2003   Fixed duplicating GUIDs.  Sometimes they duplicate if multiples are created
#                    in the same millisecond (it checks the last 100 GUIDs now and has a larger random part)
# December 9, 2003   Fixed MAX_RANDOM, which was going over sys.maxint
# November 21, 2005  Changed to pack the guid using 64 bit encoding 0-9, A-Z, a-z, @, [
#                    Now, the guid is only 16 bytes long.
#

import math
import random
import socket
import os, sys
import time
import threading
import string, struct

import getopt

# The size of the circular queue.  Larger sizes give more assurance for uniqueness.
# Smaller sizes take less memory and are a tiny bit faster
QUEUE_SIZE = 100


#############################
###   global module variables

MAX_RANDOM = sys.maxint # converted to hex goes to 8 chars (at least, in Python 2.3)
rand = random.Random()
ip = ''
lock = threading.RLock()
lastguid = ''
try:
  ip = socket.gethostbyname(socket.gethostname())
except (socket.gaierror): # if we don't have an ip, default to someting in the 10.x.x.x private range
  ip = '10'
  for i in range(3):
    ip += '.' + str(rand.randrange(1, 254))
hexip = ''.join(["%04x" % long(i) for i in ip.split('.')]) # leave space for ip v6 (65K in each sub)
ipaddrStr = socket.inet_aton(ip)
(ipaddr, ) = struct.unpack(">I", ipaddrStr)

def encode64Char(i):
  if i<10: return chr(i+48)
  elif i<38: return chr(i-10+64)
  elif i<64: return chr(i-38+97)
  raise Error
  
def pack64(i, bytes=6):
  parts = []
  for j in range(bytes):
    a = i & 0x3f
    i = (i >> 6)
    parts.append(encode64Char(a))
  parts.reverse()
  p = string.join(parts, "")
  return p

def decode64Char(c):
  n = ord(c)
  if n>=48 and n<=58:
    return n-48
  elif n >= 64 and n <= 91:
    return n - 54
  elif n >= 97 and n <= 122:
    return n - 59
  raise Error

def unpack64(s, bytes=6):
  i = 0L

  for j in range(bytes):
    c = s[j]
    a = decode64Char(c)
    i = i | a
    if j < bytes-1:
      i = i << 6
  return i

if 0:
  for i in range(64):
    c = encode64Char(i)
    j = decode64Char(c)
    if i != j:
      print i, c, j
  
ipaddr64 = pack64(ipaddr)

#######################################
###   A simple circular set
###   to ensure we don't duplicate
###   GUIDs in the same millisecond
  
class CircularSet:
  '''A circular set.  A set that maxes at a given size, replacing the oldest element after maximum size.
     This implementation is NOT thread safe.  (generate() below is thread safe, though)
  '''
  def __init__(self):
    self.queue = []
    self.queue_map = {} # for efficiency, we keep a map of everything
    self.queueindex = 0

  def add(self, val):
    '''Adds a value to the queue'''
    # check to see if we have this value.  If so, throw an exception
    assert not self.queue_map.has_key(val), 'This value is already in the set!'
    
    # add the new one to the list
    if len(self.queue) > self.queueindex:
      # first remove the previous key at this location
      del self.queue_map[self.queue[self.queueindex]]
      self.queue[self.queueindex] = val
    else:
      self.queue.append(val)
      
    # now add to the map for efficiency
    self.queue_map[val] = val
    
    # increment the queue index
    self.queueindex += 1
    if self.queueindex >= QUEUE_SIZE:
      self.queueindex = 0
      
queue = CircularSet()
  
#################################
###   Public module functions


def generate(time_t = None):
  '''Generates a new guid'''
  global lock, queue  # since we modify the module variable
  if time_t > 1136102400:
    raise ValueError, "time_t is too large %s" % time_t

  try:
    lock.acquire() # can't generate two guids at the same time
    while 1:
      # time part
      if time_t == None:
        t = long(time.time() * 100)
      else:
        t = time_t * 100

      # random part
      r = int(rand.random() * MAX_RANDOM) & 0xffff
      n = 0L | (long(t)<<48) | (ipaddr<<16) | r
      guid = pack64(n, bytes=16)

      try:
        queue.add(guid)  # throws the AssertionError if this GUID is a duplicate of the queue
        return guid
      except AssertionError: # signals we already have this GUID in the queue
        pass
  finally:
    lock.release()
    
InvalidGUID = "Invalid GUID"

def extract_time(guid):
  '''Extracts the time portion out of the guid and returns the 
     number of seconds since the epoch as a float'''
  if len(guid) != 16: raise InvalidGUID
  n = unpack64(guid, bytes=16)
  t = long(n >> 48) / 100
  return t


def extract_ip(guid):
  '''Extracts the ip portion out of the guid and returns it
     as a string like 10.10.10.10'''

  if len(guid) != 16: raise InvalidGUID

  n = unpack64(guid, bytes=16)
  n = n >> 16
  n = n & 0xffffffffL

  ip = struct.pack(">L", n)
  ipaddrStr = socket.inet_ntoa(ip)
  return ipaddrStr


def extract_random(guid):
  '''Extracts the random bits from the guid (returns the bits in decimal)'''
  if len(guid) != 16: raise InvalidGUID

  n = unpack64(guid, bytes=16)
  r = int(n & 0xffff)
  return r


### TESTING OF GUID CLASS ###
def test():
  pass

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

  guids = []
  if len(args) == 0:
    guids.append(generate())
  else:
    for arg in args:
      guids.append(arg)

  for guid in guids:
    print "GUID:", guid
    print "Time:", time.strftime('%a, %d %b %Y %H:%M:%S', time.localtime(extract_time(guid)))
    print "IP:  ", extract_ip(guid)
    print "Rand:", extract_random(guid)
    print


if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)

  
