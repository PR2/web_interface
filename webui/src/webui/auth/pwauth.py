#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""


import os, sys, string, time, getopt
from pyclearsilver.log import *

import random
#import fcrypt as crypt
import crypt

def cryptPassword(password):
  # genereate new password hash
  salt = chr(random.randint(65,122)) + chr(random.randint(65,122))
  pwhash = crypt.crypt(password,salt)
  return pwhash

def checkPassword(password, pw_hash):
  new_pw_hash = crypt.crypt(password,pw_hash[:2])
  if new_pw_hash != pw_hash: 
    warn("new_pw_hash", repr(new_pw_hash), repr(pw_hash))
    return 0
  return 1


def encode_digest(digest):
    hexrep = []
    for c in digest:
        n = (ord(c) >> 4) & 0xf
        hexrep.append(hex(n)[-1])
        n = ord(c) & 0xf
        hexrep.append(hex(n)[-1])
    return ''.join(hexrep)

def decode_digest(digest):
  code = []
  for n in range(0, len(digest), 2):
    a = chr(string.atoi(digest[n:n+2], 16))
    code.append(a)
  return string.join(code, '')
    
  
    

def mungePassword(password):
  import zlib
  mpw = zlib.compress(password)

  mpw = encode_digest(mpw)

  mpw = list(mpw)
  mpw.reverse()
  mpw = string.join(mpw, '')
  return mpw

def unmungePassword(mpassword):
  import zlib

  mpassword = list(mpassword)
  mpassword.reverse()
  mpassword = string.join(mpassword, '')

  mpassword = decode_digest(mpassword)

  password = zlib.decompress(mpassword)
  return password
  
  
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
