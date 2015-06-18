#!/usr/bin/env python
"""
usage: %(progname)s [--debug]
"""

# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

PKG_NAME = 'rosweb'
import roslib; roslib.load_manifest(PKG_NAME)

import getopt
import cStringIO
import os
import signal
import string
import sys
import threading
import time
import gzip
import traceback
import BaseHTTPServer
#import logging

import BaseHTTPServer, select, socket, SocketServer, urlparse
SocketServer.ThreadingMixIn.daemon_threads = True

import Cookie
import cgi

import roslib.scriptutil
import rospy
import rosservice
import rosjson

from rosservice import ROSServiceException
from genpy.message import MessageException
from roslib import rosenv

import roslib.message
import roslib.rospack

class ROSWebException(Exception): pass

def splitTopic(topic):
  parts = topic.split(":", 2)
  topic = parts[0]
  subtopic = None
  params = None
  if len(parts) >= 2: 
    subtopic = parts[1]
  if len(parts) >= 3:
    params = parts[2]
  return topic, subtopic, params


## Factory for ROSWebTopic instances
class RWTFactory(object):
    def __init__(self, webserver):
        self.map = {}
        self.lock = threading.Lock()

        self.webserver = webserver

        self.counter = 1
        self.cond = threading.Condition()

    def close(self):
      try:
        self.lock.acquire()
        for topic in self.map.keys():
          rwt = self.map[topic]
          rwt.factory = None
          rwt = None
          del self.map[topic]
      finally:
        self.lock.release()

    def unsubscribe(self, topic):
      try:
        self.lock.acquire()
        if topic in self.map:
          del self.map[topic]
      finally:
        self.lock.release()

    def subscribe(self, topic):
        maintopic, subtopic, params = splitTopic(topic)
        main_rwt = None
        try:
            self.lock.acquire()
            if topic in self.map:
                return self.map[topic]
            main_rwt = ROSWebTopic(maintopic, self)
            self.map[maintopic] = main_rwt
            rwt = main_rwt
            if subtopic:
              handlerClass = self.webserver._subtopics["%s:%s" % (maintopic, subtopic)]
              rwt = handlerClass(topic, self, main_rwt)
              self.map[topic] = rwt
        finally:
            self.lock.release()
        return main_rwt

    def get(self, topic):
      return self.map.get(topic, None)

    def registerVirtualTopic(self, topic, thread):
      vt = VirtualTopic(topic, self, thread)
      try:
        self.lock.acquire()
        self.map[topic] = vt
        vt.init()
      finally:
        self.lock.release()


class ROSWebTopic(object):
    def __init__(self, topic, factory):
        self.topic = topic
        self.initialized = False
        self.sub = None
        self.factory = factory

        self.last_message = None
        self.messages = []
        self.subtopics = []

    def init(self):
        if self.initialized: return

        try:
            self.factory.cond.acquire()
            m = roslib.scriptutil.get_master()

            code, _, topics = m.getPublishedTopics('/'+PKG_NAME, '/')
            if code != 1:
              raise ROSWebException("unable to communicate with master")
            
            for t, topic_type in topics:
                if t == self.topic: break
            else:
                raise ROSWebException("%s is not a published topic"%self.topic)

            msg_class = roslib.message.get_message_class(topic_type)

            #print "RosWebtopic", self.topic, msg_class
            try:
              self.sub = rospy.Subscriber(self.topic, msg_class, self.topic_callback)
            except:
              print self.topic, msg_class
              raise 
            self.initialized = True
        except ROSWebException:
            raise
        except Exception, e:
            # fatal for now
            rospy.signal_shutdown(str(e))
            raise
        finally:
            self.factory.cond.release()

    def close(self):
      self.factory = None
      for rwt, subtopic in self.subtopics:
        subtopic.close()

    def topic_callback(self, data):
        try:
            self.factory.cond.acquire()

            self.messages.append((self.factory.counter, data))
            self.factory.counter = self.factory.counter + 1
            self.messages = self.messages[-1:]

            for rwt, subtopic in self.subtopics:
              rwt.subtopic_callback(data)

            self.factory.cond.notifyAll()
        finally:
            self.factory.cond.release()

    def subscribeSubTopic(self, rwt, subtopic):
      self.subtopics.append((rwt, subtopic))
      

class ROSWebSubTopic(object):
    def __init__(self, topic, factory, main_rwt):
      self.factory = factory
      self.topic = topic

      self.messages = []
      self.maintopic, self.subtopic, params = splitTopic(topic)
      main_rwt.subscribeSubTopic(self, self.subtopic)

    def init(self):
      pass

    def subtopic_callback(self, msg):
      ## no locks needed since this will be called from the main topic

      newmsg = self.transform(msg)

      if not newmsg: return

      self.messages.append((self.factory.counter, newmsg))
      self.factory.counter = self.factory.counter + 1
      self.messages = self.messages[-1:]
      
      
    

class VirtualTopic(object):
  def __init__(self, topic, factory, thread, intopics=[]):
    self.factory = factory
    self.topic = topic
    self.intopics = intopics

    self.messages = []

    self.thread = thread
    self.initialized = False

  def init(self):
    if not self.initialized:
      for _topic in self.intopics: 
        self.factory.subscribe(_topic)

      self.initialized = True
      self.thread.setCallback(self.callback)
      self.thread.start()

  def callback(self, data):
    try:
      self.factory.cond.acquire()

      self.messages.append((self.factory.counter, data))
      self.factory.counter = self.factory.counter + 1
      self.messages = self.messages[-3:]

      self.factory.cond.notifyAll()
    finally:
      self.factory.cond.release()
    

class ROSWebHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def __init__(self, *args):
        self.failed_topics = []
        BaseHTTPServer.BaseHTTPRequestHandler.__init__(self, *args)

    def subscribe(self, topic):
        rwt = self.server.factory.subscribe(topic)
        try:
          rwt.init()
        except ROSWebException, e:
          self.server.factory.unsubscribe(topic)
          if topic not in self.failed_topics:
            rospy.logwarn("Cannot subscribe to %s" % topic)
            self.failed_topics.append(topic)
          #traceback.print_exc()
          return None
        return rwt

    def _do_GET_topic(self, topics, since, nowait=False, callback=None):
        rwttopics = []

        for topic in topics:
          rwt = self.server.factory.get(topic)
          if rwt:
            rwttopics.append((topic, rwt))
          else:
            rwt = self.subscribe(topic)
            if rwt: rwttopics.append((topic, rwt))

        if len(rwttopics) == 0:
          rospy.logwarn("no valid topics")
          self.send_failure()
          return

        (messages, lastSince) = self._get_messages(rwttopics, since, nowait)

        buf = cStringIO.StringIO()
        buf.write("{")
        buf.write('"since": %s,' % lastSince)
        buf.write('"msgs": [')
        _i = 0
        for (topic, msg) in messages:
          _i = _i + 1
          if _i > 1: buf.write(',')
          buf.write("{")
          buf.write('"topic": "%s",' % topic)
          buf.write('"msg": ')
          buf.write(rosjson.ros_message_to_json(msg))
          buf.write("}")
        buf.write(']')
        buf.write('}')

        buf = buf.getvalue()

        self.send_success(buf, callback)


    def _get_messages(self, rwttopics, since, nowait=False):
        messages = []
        lastSince = 0
        try:
            self.server.factory.cond.acquire()
            # timeout?

            if since > self.server.factory.counter:
              ## reset the since if it is invalid.
              rospy.logerr("invalid since counter, reseting... %s" % (since, ))
              since = 0

            ## check for some old messages first
            lastSince = since
            for topic, rwt in rwttopics:
              if not rwt: continue
              for t,msg in rwt.messages:
                if t > since:
                  messages.append((topic, msg))
                  lastSince = max(t, lastSince)

            if nowait:
              ## do not wait for message, for grabbing messages
              return messages, lastSince
                
            ## if no old messages, then wait for a new one.
            while not messages:
              self.server.factory.cond.wait()

              # get the last message under lock
              for topic, rwt in rwttopics:
                if not rwt: continue
                for t,msg in rwt.messages:
                  if t > lastSince:
                    messages.append((topic, msg))
                    lastSince = max(t, lastSince)
        finally:
            self.server.factory.cond.release()
        return (messages, lastSince)


    def _connect_to(self, netloc, soc):
        i = netloc.find(':')
        if i >= 0:
          host_port = netloc[:i], int(netloc[i+1:])
        else:
            host_port = netloc, 80
#        print "\t" "connect to %s:%d" % host_port
        try: soc.connect(host_port)
        except socket.error, arg:
          try: msg = arg[1]
          except: 
            msg = arg
            self.send_error(404, msg)
            return 0
        return 1

    def _send_responsepage(self, retcode=200, buf = "{}", callback = None):
      try:
        self.send_response(retcode)
        if callback:
          #buf = callback + "(" + buf + "); var _xsajax$transport_status=200;"
          buf = callback + "=" + buf + ";\nvar _xsajax$transport_status=200;"
          self.send_header('Content-Type', 'text/javascript')
        else:
          self.send_header('Content-Type', 'application/json')

        if len(buf) > 500:
          acceptEncoding = self.headers.get('Accept-Encoding', '')
          if acceptEncoding.find('gzip') != -1:
            zbuf = cStringIO.StringIO()
            zfile = gzip.GzipFile(None, "wb", 9, zbuf)
            zfile.write(buf)
            zfile.close()

            nbuf = len(buf)
            buf = zbuf.getvalue()
            self.send_header('Content-encoding', 'gzip')

        self.send_header('Content-Length', str(len(buf)))

        if self.headers.get("Connection", '').lower() == "keep-alive":
          self.send_header('Connection', 'keep-alive')

        self.send_header("Access-Control-Allow-Origin", "*")  #"http://ta110.willowgarage.com")

        self.end_headers()


        self.wfile.write(buf)
      except socket.error, (ecode, reason):
        if ecode == 32:
          print ecode, reason
        else:
          raise
                        

    def send_success(self, buf = "{}", callback=None):
      self._send_responsepage(200, buf, callback)

    def send_failure(self, buf = "{}", callback=None):
      self._send_responsepage(404, buf, callback)

    def send_forbidden(self, buf = "{}", callback=None):
      self._send_responsepage(403, buf, callback)

    def address_string(self):
      host, port = self.client_address[:2]
      return host

    def handle_ROS(self, path, qdict):
      cstring = self.headers.get('Cookie', '')

      cookie = Cookie.BaseCookie()
      cookie.load(cstring)

      path_parts = path.split("/")

      username = ""

      cmd = path_parts[2]

      ## parse out the username from the cookie
      if cookie.has_key('MB_L1'):
        l1 = cookie['MB_L1'].value
        if l1.startswith("V1/"):
          code = l1[3:]
          parts = code.split(":", 1)
          username = parts[0]

      ## Make a service call
      ##   Args can be passed as an ordered list of parameters:
      ##     /ros/service/<service_name>?args=<arg1>&args=<arg2>&<...>
      ##   or as a dictionary of named parameters
      ##     /ros/service/<service_name>?json=<json-encoded dictionary of args>
      if cmd == "ping":
        self.send_success("{'ping': %f}" % time.time())

      if cmd == "service":
        name = "/".join(path_parts[3:])
        callback = qdict.get("callback", [''])[0]

        try:
          service_class = rosservice.get_service_class_by_name("/" + name)
        except ROSServiceException:
          self.send_failure()
          return
          
        request = service_class._request_class()

        args = qdict.get('args', None)
        if not args:
          args = qdict.get('json', ['{}'])[0]
          args = eval(args)
        try:
          now = rospy.get_rostime()
          keys = { 'now': now, 'auto': rospy.Header(stamp=now) }
          roslib.message.fill_message_args(request, args, keys=keys)
        except MessageException:
          raise ROSServiceException("Not enough arguments to call service.\n"+\
              "Args should be: [%s], but are actually [%s] " %
              (roslib.message.get_printable_message_args(request), args))

        rospy.logdebug("service call: name=%s, args=%s" % (name, args))
        rospy.wait_for_service(name)
        service_proxy = rospy.ServiceProxy(name, service_class)
        try:
          msg = service_proxy(request)
        except rospy.ServiceException:
          self.send_failure()
          return
        msg = rosjson.ros_message_to_json(msg)

        rospy.logdebug("service call: name=%s, resp=%s" % (name, msg))
        self.send_success(msg, callback=callback)
        return

      if cmd == "publish":
        topic = string.join(path_parts[3:], "/")

        callback = qdict.get("callback", [''])[0]
        topic_type = qdict.get("topic_type", [''])[0]

        message_klass = roslib.message.get_message_class(topic_type)

        message = message_klass()

        args = qdict.get('args', None)
        if not args:
          args = qdict.get('json', ['{}'])[0]
          args = eval(args)
        try:
          now = rospy.get_rostime()
          keys = { 'now': now, 'auto': rospy.Header(stamp=now) }
          roslib.message.fill_message_args(message, args, keys=keys)
        except MessageException:
          raise ROSServiceException("Not enough arguments to call service.\n"+\
              "Args should be: [%s], but are actually [%s] " %
              (roslib.message.get_printable_message_args(message), args))

        rospy.logdebug("publish: topic=%s, topic_type=%s, args=%s" % (topic, topic_type, args))

        pub = rospy.Publisher(topic, message_klass, latch=True)

        try:
          pub.publish(message)
        except rospy.ServiceException:
          self.send_failure()
          return

        self.send_success(callback=callback)
        return

      if cmd == "receive":
        since = int(qdict.get('since', ['0'])[0])
        topics = qdict.get("topic", "")
        callback = qdict.get("callback", [''])[0]
        nowait = int(qdict.get('nowait', ['0'])[0])
        self._do_GET_topic(topics, since, nowait=nowait, callback=callback)
        return

    
      
    def do_GET(self):
        if get_robot_type().startswith("texas"):
          try:
            stored_cookie = open("/var/ros/user_cookie.dat").read()
            request_cookie = self.headers.get('Cookie', '')
            if stored_cookie.find(request_cookie) == -1:
              rospy.logwarn("cookie did not match stored cookie \n%s\n%s" % (stored_cookie, request_cookie))
              self.send_forbidden()
              return
          except:
            self.send_forbidden()
            return

        (scm, netloc, path, params, query, fragment) = urlparse.urlparse(
            self.path, 'http')
        qdict = cgi.parse_qs(query)
        pdict = cgi.parse_qs(str(self.rfile._rbuf))
        qdict.update(pdict)

        self.log_request()

        for handler in self.server._handlers:
          if path.startswith(handler['url']):
            handler['handler'](self, path, qdict)
            return

        netloc = "localhost:80"
        scm = "http"

        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
          if self._connect_to(netloc, soc):
                soc.send("%s %s %s\r\n" % (
                    self.command,
                    urlparse.urlunparse(('', '', path, params, query, '')),
                    self.request_version))
                self.headers['Connection'] = 'close'
                del self.headers['Proxy-Connection']
                for key_val in self.headers.items():
                    soc.send("%s: %s\r\n" % key_val)
                soc.send("\r\n")
                self._read_write(soc)
        finally:
            soc.close()
            self.connection.close()

      
    def _read_write(self, soc, max_idling=20):
        iw = (self.connection, soc)
        ow = ()
        count = 0

        while 1:
            count += 1
            (ins, _, exs) = select.select(iw, ow, iw, 3)
            if exs: break
            if ins:
              for i in ins:
                if i is soc:
                  out = self.connection
                elif i is self.connection:
                  out = soc
                else:
                  print "error"
                data = i.recv(8192)
                if data:
                  bytes_sent = 0
                  while bytes_sent != len(data):
                    n = out.send(data[bytes_sent:])
                    bytes_sent += n
                  count = 0
            else:
                pass
            if count == max_idling: break

    do_POST = do_GET
    do_OPTIONS = do_GET

    def log_message(self, format, *args):
      self.server.accesslog_fp.write("%s - - [%s] %s\n" %
                              (self.address_string(),
                               self.log_date_time_string(),
                               format%args))
      self.server.accesslog_fp.flush()

      
class MasterMonitor:
  def run(self):
    global running
    master = roslib.scriptutil.get_master()
    while running:
      try:
        master.getSystemState('/rostopic')
      except:
        import traceback
        #traceback.print_exc()
        return
      time.sleep(1)

class ThreadingHTTPServer (threading.Thread,
                           SocketServer.ThreadingMixIn,
                           BaseHTTPServer.HTTPServer): 
  def __init__(self, hostport, handlerClass):
    threading.Thread.__init__(self)
    SocketServer.TCPServer.__init__(self, hostport, handlerClass)

    self.factory = RWTFactory(self)

    logfn = os.path.join(rosenv.get_log_dir(), "access_log")
    self.accesslog_fp = open(logfn, "a+")

    self._handlers = []
    self.register_handler('/ros', ROSWebHandler.handle_ROS)

    self._subtopics = {}

    self.__load_plugins()
    self.__registerVirtualTopics()

  def register_handler(self, urlprefix, handler):
    h = {'url': urlprefix, 'handler': handler}
    self._handlers.append(h)

  def register_subtopic(self, topic, handlerClass):
    self._subtopics[topic] = handlerClass

  def __load_plugins(self):
    plugins = roslib.rospack.rospack_plugins(PKG_NAME)
    for (package, plugin) in plugins:
      try:
        roslib.load_manifest(package)
        mods = plugin.split('.')
        mod = __import__(plugin)
        for sub_mod in mods[1:]:
          mod = getattr(mod, sub_mod)

        mod.config_plugin(self)

      except Exception, reason:
        rospy.logerr("plugin %s got exception %s" % (plugin, reason))


  def __registerVirtualTopics(self):
    import topics
    ut = topics.TopicThread()
    ut.setDaemon(True)
    self.factory.registerVirtualTopic("/topics", ut)

    import params
    ut = params.ParamThread()
    ut.setDaemon(True)
    self.factory.registerVirtualTopic("/parameters", ut)

  def run(self):
    while 1:
      self.handle_request()

def rosweb_start():
    web_server = None
    mm = None

    try:
      rospy.init_node(PKG_NAME, disable_signals=True)

      web_server = ThreadingHTTPServer(('', 8068), ROSWebHandler)

      web_server.setDaemon(True)
      web_server.start()
      rospy.loginfo("starting Web server")

      mm = MasterMonitor()
      mm.run()

    finally:
      rospy.signal_shutdown(PKG_NAME + ' exiting')
      if web_server: web_server.server_close()

error_log_fp = None

def signal_handler(signal, frame):
  global running
  running = False

def usage(progname):
  print __doc__ % vars()

  
def get_robot_type():
  try:
    return open(os.path.join("/etc/ros/env", "ROBOT")).read().strip()
  except IOError:
    return "desktop"

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "debug"])

  debugFlag = False
  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugFlag = True

  global running, error_log
  running = True
  #if debugFlag:
  #  logging.basicConfig(level=logging.DEBUG)
  #  pass
  #else:
  #  logging.basicConfig(filename="error_log", level=logging.DEBUG)

  signal.signal(signal.SIGINT, signal_handler)


  while running:
    rospy.loginfo("starting")
    try:
      rosweb_start()
    except:
      import traceback
      traceback.print_exc()
    time.sleep(1)

  if error_log_fp: error_log_fp.close()

if __name__ == '__main__':
  main(sys.argv, sys.stdout, os.environ)
