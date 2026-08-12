#!/usr/bin/env python3
import subprocess

script = ["python2.7", "happy.py", "192.168.0.100", "5"]
process = subprocess.Popen(" ".join(script), shell=True)