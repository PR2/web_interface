#! /usr/bin/env python

import subprocess
import threading
import time

import roslib
from roslib import scriptutil

class Topics(roslib.message.Message):
  __slots__ = ('pubtopics', 'subtopics','nodes')
  def __init__(self, topics):
    self.pubtopics, self.subtopics, self.nodes = topics

def succeed(args):
    code, msg, val = args
    if code != 1:
        raise RosTopicException("remote call failed: %s"%msg)
    return val


class TopicThread(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)

    self.callback = None

  def setCallback(self, callback):
    self.callback = callback

  def getTopics(self):
    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = scriptutil.get_master()
    topic = None

    state = succeed(master.getSystemState('/rostopic'))
    pubs, subs, _ = state

    pub_topics = succeed(scriptutil.get_master().getPublishedTopics('/rostopic', '/'))    
    pubtopics = [t for t,_ in pubs]
    pubtopics.sort()

    subtopics = [t for t,_ in subs]
    subtopics.sort()

    nodes = []

    for s in state:
        for t, l in s:
            nodes.extend(l)

    nodes = set(nodes)
    nodes = list(nodes)
    nodes.sort()

    topics = (tuple(pubtopics), tuple(subtopics), tuple(nodes))

    return topics

  def run(self):
    lasttopics = ()
    while 1:
      try:
        topics = self.getTopics()
        if topics != lasttopics:
          lasttopics = topics
          self.callback(Topics(topics))
      except:
        pass
      time.sleep(3)


