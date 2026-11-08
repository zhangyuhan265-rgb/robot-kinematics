import os
import sys

sys.path.insert(0, os.path.expanduser("~/projects/07_机器人学/nrc_sdk"))
import nrc_interface as nrc

print("OK")
print("file:", nrc.__file__)
print("py:", sys.version)
