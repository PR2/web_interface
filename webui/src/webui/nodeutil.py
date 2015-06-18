#! /usr/bin/env python

import os
import socket
import xmlrpclib

PKG = 'webui' # this package name
import roslib; roslib.load_manifest(PKG) 
import rospy
from roslib import scriptutil
from roslib.names import make_caller_id

def _remote_call(args):
  code, msg, val = args
  if code != 1:
    raise Exception("remote call failed: %s"%msg)
  return val

_caller_apis = {}
def get_api_uri(master, node_name, caller_id):
  caller_api = _caller_apis.get(caller_id, None)
  if not caller_api:
      try:
          code, msg, caller_api = master.lookupNode(caller_id, node_name)
          if code != 1:
              return None
          else:
              _caller_apis[caller_id] = caller_api
      except socket.error:
          raise Exception("Unable to communicate with master!")
  return caller_api


def node_info(node):
  caller_id = make_caller_id('nodeutil-%s'%os.getpid())
  node_name = node #scriptutil.script_resolve_name(caller_id, node)
  master = scriptutil.get_master()

  # go through the master system state first
  try:
    state = _remote_call(master.getSystemState(caller_id))
  except socket.error:
    raise Exception("Unable to communicate with master!")
      
  pubs = [t for t, l in state[0] if node_name in l]
  subs = [t for t, l in state[1] if node_name in l]
  srvs = [t for t, l in state[2] if node_name in l]  

  results = {
    "publications": pubs,
    "subscriptions": subs,
    "services": srvs
  }
  node_api = get_api_uri(master, node_name, caller_id)
  if not node_api:
    results["error"] = "cannot contact [%s]: unknown node"%node_name

  return results

def topic_info(topic):
  caller_id = make_caller_id('nodeutil-%s'%os.getpid())
  master = scriptutil.get_master()

  # go through the master system state first
  try:
    state = _remote_call(master.getSystemState(caller_id))
  except socket.error:
    raise IOException("Unable to communicate with master!")
      
  pubs, subs, _ = state
  publists = [publist for t, publist in pubs if t == topic]
  sublists = [sublist for t, sublist in subs if t == topic]

  #pub_topics = _succeed(master.getPublishedTopics(caller_id, '/'))

  if publists == []:
    return {
      "error": "This topic does not appear to be published yet."
    }
  elif sublists == []: 
    return {  
        "publishers": publists[0], 
        "subscribers": [] 
    } 
  else:
    return {
      "publishers": publists[0],
      "subscribers": sublists[0]
    }

def main():
  print node_info("/fake_wifi_ddwrt")
  print "----"
  print topic_info("/ddwrt/accesspoint")


if __name__ == "__main__":
  main()
