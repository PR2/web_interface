#! /usr/bin/env python

import subprocess
import threading
import time
import roslib

class Users(roslib.message.Message):
  __slots__ = ('users',)
  def __init__(self, users):
    self.users = users

class UsersThread(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)

    self.callback = None

  def setCallback(self, callback):
    self.callback = callback

  def getUsers(self):
    p = subprocess.Popen(['users'], stdout=subprocess.PIPE)
    line = p.stdout.readline().strip()
    parts = line.split()
    users = {}
    map(users.__setitem__, parts, [])
    users = users.keys()
    users.sort()
    users = tuple(users)

    return users

  def run(self):
    lastusers = ()
    while 1:
      try:
        users = self.getUsers()
        if users != lastusers:
          lastusers = users
          self.callback(Users(users))
      except:
        pass
      time.sleep(3)


