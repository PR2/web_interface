#!/neo/opt/bin/python


import ihooks, os, sys
from pyclearsilver.log import *

#ihooks.install(AutoReload())

class AutoReload(ihooks.ModuleImporter):
  def __init__(self):
    ihooks.ModuleImporter.__init__(self)
    self.datestamps = {}

  def import_module(self,name,globals={},locals={},fromlist={}):
#    warn("import_module", name)
    testpath = name + ".py"

    # stat the file if we have it
    if os.path.isfile(testpath):
      stinfo = os.stat(testpath)
    else:
      stinfo = None

    if sys.modules.has_key(name):
#      warn("reimporting", testpath)
      # already imported
      m = ihooks.ModuleImporter.import_module(self,name,globals,locals,fromlist)
      try: 
        testpath = m.__file__
        if os.path.isfile(testpath):
          stinfo = os.stat(testpath)
      except AttributeError: pass

      if stinfo:

        stored_time = self.datestamps.get(testpath,0)
        if stored_time < stinfo.st_mtime:
                                        
          self.datestamps[testpath] = stinfo.st_mtime
          if stored_time:
            warn("---------------------", name, "changed reloading", stored_time, stinfo.st_mtime, os.getcwd())
            
            reload(sys.modules[name])
    else :
#      warn("loading for the first time", testpath)
      if stinfo:
        self.datestamps[testpath] = stinfo.st_mtime
      m = ihooks.ModuleImporter.import_module(self,name,globals,locals,fromlist)
      try:
        testpath = m.__file__
        stinfo = os.stat(testpath)
        if os.path.isfile(testpath):
          self.datestamps[testpath] = stinfo.st_mtime
      except AttributeError:
        pass

    return m


warn("**********************8 installing autoreload")
ihooks.install(AutoReload())        
