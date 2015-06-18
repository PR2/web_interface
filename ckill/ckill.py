#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## 

"""
usage: %(prog)s [list|kill|save|halt]
"""

PKG = 'ckill' # this package name
NAME = 'ckill'

import os, sys, string, time
import re
import subprocess
import UserDict
from optparse import OptionParser
import pwd

class ProcessError(Exception):
  pass

def spawn(cmd):
  p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

  stdout, stderr = p.communicate()
  if p.returncode != 0:
    raise ProcessError, "Error: %s" % stderr

  return stdout

class Process:
  def __init__(self, host, username, uid, pid, ppid, cmd, args):
    self.host = host
    self.pid = pid
    self.ppid = ppid
    self.username = username
    self.uid = int(uid)
    self.cmd = cmd
    self.arg_line = args
    self.args = args.split()


    if self.cmd.startswith("python"):
      self.cmd2 = string.join(self.args[:2])
    else:
      self.cmd2 = self.args[0]

  def display(self):
    print "%-6s %-6s %-6s %-12s %-6s %-20s -- %s" % (self.host, self.pid, self.ppid, self.username, self.uid, self.cmd, self.cmd2)

  def __repr__(self):
    return "<Process %s %s %s %s %s %s>" % (self.host, self.pid, self.ppid, self.username, self.uid, self.cmd)

class ProcessList(UserDict.UserDict):
  def __init__(self, hosts):
    UserDict.UserDict.__init__(self)
    self.hosts = hosts
    self.fetch()

  def fetch(self):
    self.clear()

    for host in self.hosts:
      self.fetch_process_list(host)

  def fetch_process_list(self, host):
    cmd = ['ps', '-e', '--no-headers', '-o', 'pid=', '-o', 'ppid=', '-o', 'user=', '-o', 'uid=', '-o', 'comm=', '-o', 'args=', ]
    ppid = None
    if host != "localhost":
      ppid = ['python', '-c', "'import os; print os.getppid();print'", ";"]
      cmd = ['ssh', host, ] + ppid + cmd

    stdout = spawn(cmd)
      
    lines = stdout.split("\n")

    thispid = None
    if ppid:
      thispid = int(lines[0].strip())

    for line in lines[1:]:
      if not line: continue

      parts = line.split(None, 5)
      if len(parts) != 6: continue

      pid = int(parts[0])
      ppid = int(parts[1])
      username = parts[2]
      uid = parts[3]
      cmd = parts[4]

      p = Process(host, username, uid, pid, ppid, cmd, parts[5])
      self[pid] = p

    if thispid:
      ## remove the ssh processes from the process table.
      p = self.get(thispid, None)
      while p:
        del self[p.pid]
        p = self.get(p.ppid, None)
    
      

class RosInit:
  def __init__(self, hosts, minuid, maxuid, whitelistfn, blacklistfn, uid, regex):
    self.whitelist = {}
    self.blacklist = {}

    self.minuid = minuid
    self.maxuid = maxuid
    self.hosts = hosts
    self.uid = uid
    self.regex = re.compile(regex)

    if whitelistfn:
      self.whitelistfn = whitelistfn
      self.whitelistpath, fn = os.path.split(self.whitelistfn)    
      self.readWhitelist(self.whitelistfn)

    if blacklistfn:
      self.blacklistfn = blacklistfn
      self.blacklistpath, fn = os.path.split(self.blacklistfn)
      self.readBlacklist(blacklistfn)

  def readWhitelist(self, fn):
    if os.path.exists(fn):
      lines = open(fn).readlines()
      for line in lines:
        if line.startswith("#include"):
          parts = line.split(' ', 1)
          ifn = parts[1].strip()
          if ifn:
            ifn = os.path.join(self.whitelistpath, ifn)
            self.readWhitelist(ifn)
        
        self.whitelist[line.strip()] = 1

  def readBlacklist(self, fn):
    lines = open(fn).readlines()
    for line in lines:
      if line.startswith("#include"):
        parts = line.split(' ', 1)
        ifn = parts[1].strip()
        if ifn:
          ifn = os.path.join(self.blacklistpath, ifn)
          self.readBlacklist(ifn)

      elif not line.startswith("#"):
        self.blacklist[line.strip()] = 1
  
  
  def kill_list(self, host):
    pl = ProcessList([host])

    ## generate a list of pids so that we don't accidentally kill this process.
    thislist = []
    if host == "localhost":
      p = pl.get(os.getpid(), None)
      while p:
        thislist.append(p.pid)
        p = pl.get(p.ppid, None)

    todo = []
    for p in pl.values():
      if p.cmd2[0] == '[' and p.cmd2[-1] == ']':  
        ## skip all kernel threads
        continue
      if p.host == "localhost" and p.pid in thislist:
        ## skip this process
        continue        
      # if we have a blacklist, filter out anything not in it 
      if self.blacklist:  
        include_cmd = False 
        for term in self.blacklist.keys():
          if re.search(term, p.cmd2):
            include_cmd = True
        if not include_cmd:
          continue
        else:
          todo.append(p)
          continue
      if p.uid < self.minuid and p.uid != self.uid:
        # skip a likely system process
        continue
      if p.uid > self.maxuid and p.uid != self.uid:
        # skip a likely system process
        continue
      if p.cmd in self.whitelist:
        ## skip whitelisted process based on 'ps -o comm='
        continue
      if p.cmd2 in self.whitelist: 
        ## skip whitelisted process based on 'ps -o args='
        continue

      # Don't kill process if it doesn't match the regex
      if self.regex.match(p.cmd) is None and self.regex.match(p.cmd2) is None:
        continue

      todo.append(p)
    
    return todo
      
  def kill(self, signal=15):
    if os.getuid() != 0:
      print "you need to be root to run this."
      return

    for host in self.hosts:
      todo = self.kill_list(host)
      todo = map(lambda x: str(x.pid), todo)

      if len(todo) == 0: continue

      cmd = ['kill']
      cmd = cmd + ["-%s" % signal]
      cmd = cmd + todo
      if host != "localhost": cmd = ['ssh', host] + cmd
      
      p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      stdout, stderr = p.communicate()
      if p.returncode != 0:
        if stderr.find("No such process") == -1:
          print "Error killing processes:", host, stderr

  def list(self):
    for host in self.hosts:
      todo = self.kill_list(host)

      for p in todo:
        #print p.host, p.pid, p.username, p.cmd2
        p.display()
    
  def save(self):
    whitelist = {}
    pl = ProcessList(self.hosts)
    for p in pl.values():
      if p.cmd2[0] == '[' and p.cmd2[-1] == ']': 
        ## skip the kernel threads
        continue
      whitelist[p.cmd2] = 1

    whitelist = whitelist.keys()
    whitelist.sort()
    for c in whitelist:
      print c
    
  def halt(self):
    for host in self.hosts:
      cmd = ['shutdown', '-h', 'now']
      if host != "localhost": cmd = ['ssh', host] + cmd

      p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      stdout, stderr = p.communicate()
      if p.returncode != 0:
        if stderr.find("No such process") == -1:
          print "Error:", host, stderr

  def reboot(self):
    for host in self.hosts:
      cmd = ['shutdown', '-r', 'now']
      if host != "localhost": cmd = ['ssh', host] + cmd

      p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      stdout, stderr = p.communicate()
      if p.returncode != 0:
        if stderr.find("No such process") == -1:
          print "Error:", host, stderr


def usage(prog):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]

  parser = OptionParser(usage="""%prog [list|kill|save|halt|reboot]

'kill' will send the signal specified by --sig to all processes.
"""

)

  parser.add_option("--blacklist", action="store", dest="blacklist", default=None, help="List of programs to always kill")
  parser.add_option("--whitelist", action="store", dest="whitelist", default=None, help="List of programs to never kill (Default: /etc/ckill/whitelist)")
  parser.add_option("--minuid", action="store", type="int", dest="minuid", default=1000, help="Never kill programs run by users below this UID (Default: 1000 -- generally considered first user)")
  parser.add_option("--maxuid", action="store", type="int", dest="maxuid", default=29999, help="Never kill programs run by users above this UID (Default: 29999 -- generally considered last user)")
  parser.add_option("--user", action="store", type="string", dest="user", default='ros', help="Explicitly include user.  (Defaults to user: ros)")
  parser.add_option("--conf", action="store", dest="confpath", default="/etc/ckill", help="Specify an alternative conf location (Default: /etc/ckill)")
  parser.add_option("--sig", action="store", dest="signum", default="TERM", help="Specify the signal to send (Default: TERM)")
  parser.add_option("--regex", action="store", dest="regex", default=".*", help="Specify a regular expression for processes to kill.")

  # Parse arguments
  (options, args) = parser.parse_args()

  sigvals = { 'HUP':1, 'SIGHUP':1, 
             'INT':2, 'SIGINT':2, 
             'QUIT':3, 'SIGQUIT':3, 
             'KILL':9, 'SIGKILL': 9, 
             'TERM':15, 'SIGTERM':15 }

  signum = 15

  try:
    signum = int(options.signum)
  except ValueError:
    try:
      signum = sigvals[options.signum]
    except KeyError:
      parser.error("Unknown signal: %s"%options.signum)

  # Require a command
  if len(args) == 0:
    parser.error("You must specify a command.")
    return

  # Read in hosts
  hostfn = os.path.join(options.confpath, "hosts")
  hosts = []
  if os.path.exists(hostfn):
    hostlines = open(hostfn).readlines()
    for host in hostlines: hosts.append(host.strip())
  else:
    hosts.append('localhost')
  
  if options.whitelist is None:
    options.whitelist = os.path.join(options.confpath, "whitelist")

  if options.user:
    try:
      uid = pwd.getpwnam(options.user).pw_uid
    except KeyError:
      # There should never be a -1 uid, so this is a safe place to default this to
      uid = -1

  ri = RosInit(hosts, options.minuid, options.maxuid, options.whitelist, options.blacklist, uid, options.regex)

  cmd = args[0]
  
  if cmd == "term":
    ri.kill(15)
  elif cmd == "kill":
    ri.kill(signum)
  elif cmd == "save":
    ri.save()
  elif cmd == "list":
    ri.list()
  elif cmd == "halt":
    ri.halt()
  elif cmd == "reboot":
    ri.reboot()
  else:
    parser.error('Uknown command: %s'%cmd)


if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
