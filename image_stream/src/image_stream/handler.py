#!/usr/bin/env python

import math
import rospy
import rosservice
import roslib

import rosweb
import threading

import cv
from cv_bridge import CvBridge

subscriptions = {}
images = {}
sub_lock = threading.Lock()

def config_plugin(context):
  context.register_handler("/image", image_handler)

def image_callback(msg, t):
  (topic, cond) = t
  bridge = CvBridge()
  img = bridge.imgmsg_to_cv(msg, "bgr8")
  data = cv.EncodeImage(".jpeg", img).tostring()
  #convert to jpeg
  cond.acquire()
  images[topic].append(data)
  cond.notifyAll()
  cond.release()

def image_handler(self, path, qdict):
  global images
  topic = qdict.get('topic', [''])[0]
  if not topic:
    return False
  m = roslib.scriptutil.get_master()
  code, _, topics = m.getPublishedTopics('/'+rosweb.PKG_NAME, '/')
  if code != 1:
    raise rosweb.ROSWebException("unable to communicate with master")

  for t, topic_type in topics:
    if t == topic:
      break
  else:
    raise rosweb.ROSWebException("%s is not a published topic" % topic)

  msg_class = roslib.message.get_message_class(topic_type)
  sub_lock.acquire()
  if topic in subscriptions:
    cond = subscriptions[topic][1]
  else:
    cond = threading.Condition()
    images[topic] = []
    subscriptions[topic] = (rospy.Subscriber(topic, msg_class, image_callback, (topic, cond)), cond)
  sub_lock.release()

  self.send_response(200)
  self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=--myboundary')
  self.end_headers()
  while 1:
    cond.acquire()
    while len(images[topic]) == 0:
      #print "waiting for an image"
      cond.wait()
    try:
      if len(images[topic]) > 0:
        data = images[topic][-1]
        images[topic] = []
        self.wfile.write('--myboundary\r\n')
        self.wfile.write('Content-Type: image/jpeg\r\n')
        self.wfile.write('Content-Length: %s\r\n' % len(data))
        self.wfile.write('\r\n')
        self.wfile.write(data)
    except:
      pass
    cond.release()

  return True

