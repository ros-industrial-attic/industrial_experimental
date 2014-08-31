
import os
import errno

def mkdir(path):
  try:
    os.mkdir(path)
  except OSError as ex:
    if ex.errno == errno.EEXIST and os.path.isdir(path):
      print("Directory: " + path + " already exists, ignoring")
      pass
    else: raise

def touch(path):
  open(path, 'a').close()
