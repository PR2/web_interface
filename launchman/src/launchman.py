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

PKG = 'launchman' # this package name
NAME = 'launchman'

import roslib; roslib.load_manifest(PKG) 

import time
import string
from optparse import OptionParser

from launchman.srv import *
from launchman.msg import *
import rospy 

import roslib.names
import roslib.network 

from roslaunch.config import ROSLaunchConfig
#from roslaunch.launch import ROSLaunchRunner
import roslaunch.parent
from roslaunch.pmon import pmon_shutdown as _pmon_shutdown
from roslaunch.xmlloader import *

from launchman import app

class AppGroup:
  def __init__(self, manager, apprunner):
    self.apprunner = None
    self.manager = manager

class AppRunner:
  def __init__(self, taskid, manager):
    self.taskid = taskid
    self.task = AppUpdate(None, None, None, None, None)
    self.app = None
    self.runner = None
    self.childGroups = []
    self.manager = manager

  def __del__(self):
    if self.manager: self.manager = None
    if self.childGroups: self.childGroups = None

  def launch(self):
    package, launch_file = string.split(self.app.launch_file, "/", 1)
    path = app.getPackagePath(package)
    print "pkgpath", path
    print "launchfile", launch_file

    fn = os.path.join(path, launch_file)
    if not os.path.exists(fn):
      return False

    self.runner = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"), [fn], is_core=False, process_listeners=(self, ))
    self.runner.start()

    return True

  def stop(self):
    if not self.runner: 
      print "no runner", self
      return
    self.runner.shutdown()
    self.runner = None

  def process_died(self, process_name, exit_code):
    rospy.logerr( "process_died, but we're ignoring this: %s, %s" % ( process_name, exit_code))
    return

    rospy.logdebug( "process_died: %s, %s" % ( process_name, exit_code))
    if self.runner:  rospy.logdebug(self.runner.pm.procs)

    if not self.runner: return

    if len(self.runner.pm.procs) == 1:
      #make sure to clean up the runner so that we don't strand roslaunch processes
      try:
        self.runner.shutdown()
      except Exception as e:
        pass

      self.runner = None
      print "ALL DONE!"
      self.manager._stopTask(self)
    else:
      if self.runner.pm.procs:
        self.task.status = "error"
        self.manager._send_status()

        print "too many processes left:", len(self.runner.pm.procs)
        for proc in self.runner.pm.procs:
          proc.stop()
        
      #make sure to clean up the runner so that we don't strand roslaunch processes
      try:
        self.runner.shutdown()
      except Exception as e:
        pass

      self.runner = None
      self.manager._stopTask(self)
      
      

  def __repr__(self):
    return "<AppRunner %s: %s %s %s>" % (self.taskid, self.app.provides, self.app.taskid, len(self.childGroups))



class TaskManager:
  def __init__(self, kill_on_conflict=False):
    self.app_update = rospy.Publisher("app_update", AppUpdate, self)
    self.app_status = rospy.Publisher("app_status", AppStatus, self)
    self._taskGroups = {}
    self._apps = {}
    self.kill_on_conflict = kill_on_conflict

  def _send_status(self):
    apps = []
    for taskid, runner in self._apps.items():
      apps.append(Application(taskid, runner.app.name, runner.task.status, runner.app.icon, str(runner.app.provides), str(runner.app.depends)))
    self.app_status.publish(apps)

  # Look for running tasks that provide any of the capabilities included 
  # in deps.  Returns a tuple: (uncovered, pgroup).   uncovered is the subset
  # of deps not provided by any running task.  pgroup 
  # is a list of AppRunner objects associated with all tasks that provide 
  # any of deps.
  def _find_providers(self, deps):
    if not deps:
      return (), []
    d = set(deps)
    pgroup = []
    for k in self._taskGroups:
      isect = set(k) & d
      if isect:
        pgroup.append(self._taskGroups[k])
        d = d - isect
    return tuple(d), pgroup

  def list_tasks(self, req):
    s = []
    for a in self._taskGroups:
      s.append(self._taskGroups[a].app.name)    
    return ListTasksResponse(s)

  def start_task(self, req):
    a = app.App(req.taskid)
    a.load()

    # Check for conflicts between this task's provides and those of 
    # running tasks.
    pgroup = None
    uncovered, pgroup = self._find_providers(a.provides)
    for runner in pgroup:
      if runner.task.taskid == req.taskid:
        print "already running"
        self._send_status()
        return StartTaskResponse("This app is already running.")

      if self.kill_on_conflict:
        self._stopTask(runner)
      else:
        return StartTaskResponse("The %s app conflicts with the %s app; please stop the %s app"%(a.name, runner.app.name, runner.app.name))

    # Check for satisfaction of this task's dependencies.
    if a.depends:
      uncovered, pgroup = self._find_providers(a.depends)
      if uncovered:
        print "no parent task group %s running" % str(uncovered)
        self._send_status()
        return StartTaskResponse("No app that provides %s is running." % str(uncovered))

    runner = AppRunner(req.taskid, self)
    runner.app = a
    self._apps[req.taskid] = runner

    if a.provides:
      self._taskGroups[a.provides] = runner

    if pgroup:
      for p in pgroup:
        p.childGroups.append(runner)

    #print "startTask [%s, %s, %s]" % (req.taskid, a.name, req.username)
    runner.task.taskid = req.taskid
    runner.task.username = req.username
    runner.task.started = rospy.get_rostime()
    runner.task.status = "starting"

#    self.app_update.publish(runner.task)
    self._send_status()

    try:
      runner.launch()
    except Exception as e:
      import traceback
      traceback.print_exc()
      runner.task.status = "error"
      self._send_status()

      self.runner = None
      self._stopTask(runner)
      return StartTaskResponse("Exception while trying to launch the %s app: \"%s\""%(req.taskid, e))
      
    runner.task.status = "running"
#    self.app_update.publish(runner.task)
    self._send_status()

    return StartTaskResponse("done")

  def showstate(self):
    print "_" * 40
    for provides, runner in self._taskGroups.items():
      print provides, runner.childGroups

  def _stopTask(self, runner):
    runner.task.status = "stopping"
#    self.app_update.publish(runner.task)
    self._send_status()

    for cgroup in runner.childGroups[:]:
      self._stopTask(cgroup)

    runner.stop()

    runner.task.status = "stopped"
    self._send_status()

    if runner.app.depends:
      # Check for tasks that we need to shut down as a consequence 
      # of stopping this task.
      uncovered, pgroup = self._find_providers(runner.app.depends)
      for p in pgroup:
        if runner in p.childGroups:
          p.childGroups.remove(runner)
  
    if runner.app.provides:
      print "removing", runner.app.provides
      del self._taskGroups[runner.app.provides]

    if runner.taskid in self._apps:
      del self._apps[runner.taskid]
      print "removing", runner.taskid


  def stop_task(self, req):
    if req.taskid not in self._apps:
      return StopTaskResponse("no such task: %s" % req.taskid)
      
    runner = self._apps[req.taskid]

    self._stopTask(runner)

    self._send_status()

    return StopTaskResponse("done")

  def status_update(self, req):
    self._send_status()
#    for provides, runner in self._taskGroups.items():
#      self.app_update.publish(runner.task)
    return StatusUpdateResponse("done")


  def peer_subscribe(self, topic_name, topic_publish, peer_publish):
    pass

  def peer_unsubscribe(self, topic_name, numPeers):
    pass


def server(kill_on_conflict):
    rospy.init_node(NAME)

    tm = TaskManager(kill_on_conflict)
    s1 = rospy.Service('start_task', StartTask, tm.start_task)
    s2 = rospy.Service('stop_task', StopTask, tm.stop_task)
    s3 = rospy.Service('status_update', StatusUpdate, tm.status_update)
    s4 = rospy.Service('list_tasks', ListTasks, tm.list_tasks)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    parser = OptionParser(usage="usage: %prog [options]", prog='launchman.py')
    parser.add_option("-k", "--kill-on-conflict", dest="kill", 
                      default=False, action="store_true", 
                      help="automatically kill conflicting tasks when starting a new task")
    options, args = parser.parse_args()
    #initialize roslaunch in main thread
    roslaunch.pmon._init_signal_handlers()
    server(options.kill)
