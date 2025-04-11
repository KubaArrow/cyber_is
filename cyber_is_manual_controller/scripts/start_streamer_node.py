#!/usr/bin/env python3

import subprocess
import os

if __name__ == '__main__':
    current_path = os.path.dirname(os.path.realpath(__file__))
    streamer_path = os.path.join(current_path, '../bash/start_streamer.sh')
    subprocess.call(['bash', streamer_path])
