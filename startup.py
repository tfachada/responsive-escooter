import os
import subprocess

source_root = f'export HDTN_SOURCE_ROOT=~/HDTN'

subprocess.run("sudo pigpiod", shell=True)
os.environ['HDTN_SOURCE_ROOT'] = '~/HDTN'
print(f"Defined source_root: {os.getenv('HDTN_SOURCE_ROOT')}")
