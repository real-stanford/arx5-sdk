"""Preview the X5 display model without connecting to hardware.

Inputs: optional --urdf, --host and --port; defaults use this checkout's X5 URDF.
Output: a read-only Viser web page; no files or robot commands are written.
Prerequisites: pip install -r conda_environments/requirements_visualization.txt.
Run: python python/examples/visualize.py --host 127.0.0.1 --port 8080
Press Ctrl+C to stop. For live feedback, import ViserViewer in the program that
already owns the controller; see the README. This example is a static preview.
"""

import argparse
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

# Match the source-checkout imports used by the existing examples.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from visualization import ViserViewer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path,
                        default=Path(__file__).resolve().parents[2] / "models" / "X5.urdf")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    config = SimpleNamespace(robot_model="X5", urdf_path=str(args.urdf), joint_dof=6,
                             base_link_name="base_link", eef_link_name="eef_link")
    state = SimpleNamespace(pos=lambda: np.zeros(6), gripper_pos=0.04, timestamp=0.0)
    try:
        with ViserViewer(robot_config=config, host=args.host, port=args.port) as viewer:
            viewer.update(state)
            # Intentionally do not refresh a static sample's timestamp.
            # The feedback status becomes Stopped, rather than pretending to be live.
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
