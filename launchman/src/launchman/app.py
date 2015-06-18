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

import sys
import yaml
import subprocess
import os.path

def getPackagePath(pkg):
  cmd = ["rospack", "find", pkg]
  pkgpath = subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0].strip()
  return pkgpath

class App:
  app_keys_lists = ('provides', 'depends')
  app_keys = ('name', 'package', 'launch_file', 'description', 
              'icon') + app_keys_lists
  def __init__(self, taskid):
    if not taskid.endswith(".app"):
      taskid = taskid + ".app"
    self.taskid = taskid
    self.package, self.app_file = self.taskid.split('/', 1)
    self.path = None

  def fn(self):
    if self.path: return self.path
    self.path = getPackagePath(self.package)
    fn = os.path.join(self.path, self.app_file)
    return fn

  def load_yaml(self):
    fn = self.fn()
    # TODO catch file system exception
    doc = yaml.load(open(fn))
    return doc

  def load(self):
    doc = self.load_yaml()
    try:
      for key in self.app_keys:
        if doc.has_key(key):
          val = doc[key]
          if val is not None:
            if type(val) != list: 
              val = val.strip()
            if key in self.app_keys_lists:
              if type(val) == list: 
                val.sort()
                val = tuple(val)
              else:
                val = tuple([val])
          setattr(self, key, val)
    except KeyError:
      print "Invalid YAML file"

