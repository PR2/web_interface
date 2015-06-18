#! /usr/bin/env python

import os, sys, string, time, string
from pyclearsilver.log import *

#debugfull()
debugoff()

import gc

try:
  import warnings
  warnings.resetwarnings()
  warnings.filterwarnings("ignore")
except ImportError:
  pass

import neo_cgi, neo_cs, neo_util

from pyclearsilver import CSPage

import mimetypes
mimetypes.init(["/etc/mime.types"])


gConfig = None
def setConfig(config):
  global gConfig
  gConfig = config

  if not hasattr(gConfig, "gRequireUsername"): gConfig.gRequireUsername = 0
  if not hasattr(gConfig, "gDataFilePaths"): gConfig.gDataFilePaths = []


def split_path(path):
  # strip off leading slash, it's no fun!
  return string.split(path, '/')[1:]


class Page:
  def __init__(self, context):
    self.context = context

  def setupvars(self):

    self.path = self.context.environ.get("PATH_INFO", '')

    script_name = self.context.environ.get("SCRIPT_NAME",'')

    ## handle the case where the site is located at '/'
    if not self.path:  
      self.path = script_name
      script_name = '/'


  def start(self):
    self.setupvars()

    self._path = self.path

    rpath = self._path
    if not rpath: rpath = '/'

    if rpath == "/": rpath = gConfig.gDefaultPage

    self.path = split_path(rpath)

    username = None

    if len(self.path) == 0:
      warn("no such path", self.path)
      self.error(404)
      return 404

    ## the url form should be:
    ##    /baseuri/username/module/script.py

    cwd = os.getcwd()
    debug("CWD", cwd)

    module = gConfig.gDefaultModule

    if hasattr(gConfig, "gDataFilePaths"):
      if self.path[0] in gConfig.gDataFilePaths:
        fn = apply(os.path.join, [cwd,] + self.path)
        return outputFile(self.context, fn)

    if gConfig.gRequireUsername:
      username = self.path[0]
      n = 1
    else:
      n = 0

    debug("self.path", self.path)

    if len(self.path) > 1:
      module = self.path[n]
      n = n + 1

    moduleRootPath = apply(os.path.join, ["mod", module])
    handlerRoot = apply(os.path.join, ["mod", module, "cgibin"])
    moduleTemplatePath = apply(os.path.join, [cwd, "mod", module, "templates"])
    systemTemplatePath = apply(os.path.join, [cwd, "templates"])
    systemJLIBPath = apply(os.path.join, [cwd, "jslib"])

    fn = apply(os.path.join, [cwd, moduleRootPath,] + self.path[n:])
    debug("fn", fn)

    ## if requesting a file, then just output it to the browser.
    if os.path.isfile(fn):
      return outputFile(self.context, fn)

    ## manage the Python module Path
    sys.path.insert(0, os.path.abspath(cwd))
    sys.path.insert(0, os.path.abspath(moduleRootPath))

    debug("sys.path", sys.path)

    handlerPath = ''

    ## find the first *real* file in the path
    ## the rest of the path becomes the pathinfo.
    m = n
    for m in range(len(self.path)-1, n-2, -1):
      handlerPath = apply(os.path.join, [handlerRoot, ] + self.path[n:m+1])
#      warn(m, len(self.path), handlerPath)

      if os.path.isdir(handlerPath):
        sys.path.insert(0, handlerPath)
      if os.path.isfile(handlerPath): break
      if os.path.isdir(handlerPath): break

    if m+1 == len(self.path):
      pathinfo = ''
    else:
      pathinfo = apply(os.path.join, self.path[m+1:])

    if os.path.isdir(handlerPath): 
      modulePath = handlerPath
      moduleFilename = module + "_index.py"
      handlerPath = os.path.join(modulePath, moduleFilename)
    else:
      modulePath, moduleFilename = os.path.split(handlerPath)

    debug(handlerPath, pathinfo)
    #warn("PATH", handlerPath, pathinfo, modulePath, moduleFilename)
    #warn("PATH", self.path)

    if not os.path.isfile(handlerPath):
      self.error(404, handlerPath + " doesn't exist")
      return 404

    import imp

    moduleName, ext = os.path.splitext(moduleFilename)

    #module = __import__(moduleName)
    module = __import__("mod.%s.cgibin.%s" % (module, moduleName), {}, {}, (None,))

    os.chdir(modulePath)
#    debug(mod)
    page = module.run(self.context)
    page.ncgi.hdf.setValue("CGI.BaseURI", gConfig.gBaseURL)

    if gConfig.gRequireUsername:
      page.ncgi.hdf.setValue("CGI.Username", username)
      page.username = username

    if hasattr(page, "checkLoginCookie"):
      if not page._pageparms.has_key("nologin"):
        try:
          page.checkLoginCookie()
        except CSPage.Redirected:
          return

    page.ncgi.hdf.setValue("CGI.PathInfo", pathinfo)
    page.clearPaths()
    page.setPaths([moduleTemplatePath, systemTemplatePath, systemJLIBPath])
    ret = page.start()

    try: 
      page.db.close()
    except AttributeError: pass

    page = None

    return ret

  def error(self, ecode, reason=None):
    import httpResponses
    message = httpResponses.gHTTPResponses[ecode]

    template = httpResponses.errorMessage_Default
    if ecode == 404:
      template = httpResponses.errorMessage_404

    hdf = neo_util.HDF()
    hdf.setValue("code", str(ecode))
    if message: hdf.setValue("message", message)
    if reason: hdf.setValue("reason", reason)

    for key,val in self.context.environ.items():
      hdf.setValue("environ." + key, str(val))

    self.context.stdout.write("Content-Type: text/html\r\n")
    self.context.setStatus(None, ecode)
    self.context.stdout.write("Status: %s\r\n" % ecode)
    self.context.stdout.write("\r\n")
    
    cs = neo_cs.CS(hdf)
    cs.parseStr(template)
    page = cs.render()

    self.context.stdout.write(page)

    warn("Error", message, reason)


def outputFile(context, fn):
  fp = open(fn, "rb")
  data = fp.read()
  fp.close()

  context.setStatus(None, 200)

  imagetype, encoding = mimetypes.guess_type(fn)
  debug("imagetype = %s  fn = %s" % (imagetype,fn))

  lines = []

  if imagetype:
    lines.append("Content-Type: %s" % imagetype)

  lines.append("Content-Length: %d" % len(data))
  
  stat = os.stat(fn)
  mtime = stat.st_mtime
  mod_str = time.strftime("%a, %d %b %Y %H:%M:%S", time.gmtime(mtime))
  
  lines.append('Last-Modified: %s GMT' % mod_str)

  expire_time = time.gmtime(time.time() + (360*24*3600))
  expire_str = time.strftime("%a, %d %b %Y %H:%M:%S", expire_time)
  lines.append('Expires: %s GMT' % expire_str)

  lines.append("\r\n")
  
  headers = string.join(lines, "\r\n")
  context.stdout.write(headers)
  
  context.stdout.write(data)

  return 200


class FakeError:
  def write(self, s):
    apache.log_error(s, apache.APLOG_STARTUP)

class ModPythonContext:
  def __init__ (self, req):

    from mod_python import apache

    self.stdout = apache.CGIStdout(req)
    self.stdin = apache.CGIStdin(req)

    self.stderr = FakeError()
    env = apache.build_cgi_env(req)

    self.environ = env

    scriptFilename = self.environ.get("SCRIPT_FILENAME", "")
    if scriptFilename:
      path, fn = os.path.split(scriptFilename)
      os.chdir(path)

  def setStatus(self, request, status):
    if request:
      request['status'] = str(status)



def handler(req):
  start = time.time()

  from mod_python import apache

  if 1:
    context = ModPythonContext(req)
    page = Page(context)
    page.mod_python_req = req

    gc.enable()
    ret = page.start()
    gc.collect()

  ret = apache.OK

  end = time.time()
  #sys.stderr.write("handler time %s\n" % int((end-start)*1000))

  return ret


def main(argv, stdout, environ):
  context = CSPage.Context()
  page = Page(context)
  page.start()


if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
