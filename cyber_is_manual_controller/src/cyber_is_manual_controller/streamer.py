import os
import subprocess
from typing import Optional

from ament_index_python.packages import get_package_share_directory


def main(args: Optional[list[str]] = None) -> None:
    pkg_share = get_package_share_directory('cyber_is_manual_controller')
    script_path = os.path.join(pkg_share, 'bash', 'start_streamer.sh')

    # Ensure executable
    try:
        st = os.stat(script_path)
        if not (st.st_mode & 0o111):
            os.chmod(script_path, st.st_mode | 0o111)
    except FileNotFoundError:
        print(f"Streamer script not found: {script_path}")
        return

    try:
        subprocess.call(['bash', script_path])
    except KeyboardInterrupt:
        pass

