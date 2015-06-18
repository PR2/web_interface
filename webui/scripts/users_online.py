#!/usr/bin/env python

PKG = 'webui' # this package name
NAME = 'users_online'

import roslib; roslib.load_manifest(PKG)

import os
import sys
import time
import urllib2
import subprocess

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty,EmptyResponse

from optparse import OptionParser
from webui import config

users = {}
active_user = None
publisher = None
subscriber = None

active_user_file = config.ACTIVE_USER_FILE #"/var/ros/active_user.dat"

INACTIVITY_LOGOUT = 120 # seconds after which to auto-logout a non-present user

def get_active_user():
    active_user = None
    mod_time = 0
    try:
        user_file = open(active_user_file, "r")
        active_user = user_file.read().strip()
        user_file.close()
        mod_time = os.stat(active_user_file).st_mtime
    except:
        pass
    return active_user, mod_time

def listener():
    while not rospy.is_shutdown():
        publish_users()
        time.sleep(5)
        
def callback(msg):
    rospy.logdebug("got user presence: %s" % msg.data)
    now = time.time()
    users[msg.data] = now
    
def remove_active_user():
    rospy.loginfo("removing active user")

    if config.get_robot_type().startswith("texas"):
        if os.path.exists(config.VALID_USER_COOKIE_FILE):
            cookie = open(config.VALID_USER_COOKIE_FILE).read()

            req = urllib2.Request(config.gLobby + "/lobby/lobby/disconnect.py?robot_name=" + config.get_robot_name())
            req.headers['Cookie'] = cookie
  
            try:
                response = urllib2.urlopen(req)
            except Exception, e:
                rospy.logwarn("error contacting lobby: %s" % e)
            
            os.remove(config.VALID_USER_COOKIE_FILE)

    if os.path.exists(active_user_file):
        os.remove(active_user_file)


    
def publish_users():
    # build list of current users online
    active_user, mod_time = get_active_user()
    online_users = []
    
    # if users == {} and active_user and time.time() - mod_time > INACTIVITY_LOGOUT:
    # we should remove the active user
    # remove_active_user()
    # active_user = ""
    
    for user in users.keys():
        last_ping = users[user]
        if user == active_user:
            # the active user file may have been updated before presence was published
            last_ping = max(last_ping, mod_time)
            rospy.logdebug("active user %s being considered; now: %f, last_ping: %f, mod_time: %f" % (user, time.time(), last_ping, mod_time))
        if time.time() - last_ping < INACTIVITY_LOGOUT:
            online_users.append(user)
        else:
            if user == active_user:
                # we should remove the active user
                remove_active_user()
                active_user = ""
            
    online_users.sort()
    out = String()
    out.data = "%s:%s" % (','.join(online_users), active_user)
    if active_user and online_users != []:
        publisher.publish(out)
        rospy.logdebug("publishing online users: %s" % out)

if __name__ == '__main__':
  try:
    rospy.init_node(NAME, anonymous=True)
    publisher = rospy.Publisher("users_online", String)
    subscriber = rospy.Subscriber("presence", String, callback)
    listener()
  except KeyboardInterrupt, e:
    pass
  print "exiting"

