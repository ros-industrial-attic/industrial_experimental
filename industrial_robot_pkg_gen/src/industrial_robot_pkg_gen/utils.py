
import os

def mkdir(path):
  try:
    os.mkdir(path)

  except OSError:
    print("Directory: " + path + " already exists, ignoring")

def touch(path):
  open(path, 'a').close()
