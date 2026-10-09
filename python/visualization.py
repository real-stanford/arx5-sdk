"""Optional, read-only Viser viewer for an existing ARX controller.

Use ``with ViserViewer(controller): ...`` around an existing control loop, or
``ViserViewer(robot_config=config)`` and ``viewer.update(joint_state)`` for replay.
Requires the visualization extra (Viser and yourdfpy). No files are written and
no controller, CAN connection, motor command, or control gains are created.
"""

import copy
import io
import logging
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def _model_directory():
    parent = Path(__file__).resolve().parent
    return parent / "models" if (parent / "models").is_dir() else parent.parent / "models"


def _load_model(config, model_dir):
    """Retain the runtime model; replace only X5 gripper visuals in memory."""
    import yourdfpy

    path = Path(config.urdf_path).resolve(strict=True)
    root = ET.parse(path).getroot()
    parents = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    names = []
    link = config.eef_link_name
    visited = set()
    while link != config.base_link_name:
        if link in visited or link not in parents:
            raise ValueError("Runtime URDF has no valid base-to-EEF chain")
        visited.add(link)
        joint = parents[link]
        if joint.get("type") != "fixed":
            names.append(joint.get("name"))
        link = joint.find("parent").get("link")
    names.reverse()
    if len(names) != config.joint_dof:
        raise ValueError("Runtime URDF chain does not match configured joint_dof")

    def resolve_mesh(filename):
        relative = Path(filename)
        candidates = [path.parent / relative, model_dir / relative]
        # Official X5 URDF references meshes/base_link.STL, but its asset is
        # stored in meshes/X5/base_link.STL. Do not change its link transform.
        if relative.as_posix() == "meshes/base_link.STL":
            candidates.append(model_dir / "meshes" / str(config.robot_model) / relative.name)
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate.resolve())
        raise FileNotFoundError(f"Missing URDF mesh {filename!r}; searched {candidates}")

    for mesh in root.iter("mesh"):
        mesh.set("filename", resolve_mesh(mesh.get("filename")))

    fingers = []
    if config.robot_model == "X5":
        gripper_path = model_dir / "X5_gripper.urdf"
        gripper = ET.parse(gripper_path).getroot()
        for mesh in gripper.iter("mesh"):
            mesh.set("filename", str((gripper_path.parent / mesh.get("filename")).resolve(strict=True)))
        wrist = root.find("link[@name='link6']")
        if wrist is None:
            raise ValueError("X5 runtime URDF is missing link6")
        for visual in list(wrist.findall("visual")):
            wrist.remove(visual)
        wrist.extend(copy.deepcopy(gripper.find("link[@name='link6']")).findall("visual"))
        existing = {element.get("name") for element in root}
        for element in gripper:
            if element.get("name") == "link6":
                continue
            if element.get("name") in existing:
                raise ValueError(f"Gripper name conflicts with runtime model: {element.get('name')}")
            root.append(element)
            if element.tag == "joint":
                fingers.append(element.get("name"))

    robot = yourdfpy.URDF.load(
        io.BytesIO(ET.tostring(root)), load_collision_meshes=False,
        filename_handler=lambda fname: fname,
    )
    if robot.base_link != config.base_link_name:
        raise ValueError("Viewer requires the configured base link to be the URDF root")
    if set(robot.actuated_joint_names) != set(names + fingers):
        raise ValueError("Unsupported actuated joints outside the base-to-EEF chain")
    return robot, names, fingers


class ViserViewer:
    """Display controller feedback at up to ``rate_hz`` (default 20 Hz).

    Pass an existing joint/Cartesian controller for automatic polling, or only
    its robot_config and call update(state). The viewer never calls recv_once,
    send_recv_once, set_* or reset_* on the controller. The caller owns hardware
    communication and must close the viewer before releasing the controller.

    Use a context manager, or start()/close(). host defaults to localhost; use
    host="0.0.0.0" to view from another computer on a trusted network. model_dir
    optionally locates SDK meshes and the display-only X5 gripper definition.
    """

    def __init__(self, controller=None, *, robot_config=None, host="127.0.0.1",
                 port=8080, rate_hz=20.0, model_dir=None):
        if (controller is None) == (robot_config is None):
            raise ValueError("Pass either controller or robot_config")
        if not np.isfinite(rate_hz) or not 0 < rate_hz <= 60:
            raise ValueError("rate_hz must be in (0, 60]")
        self._controller = controller
        self._config = controller.get_robot_config() if controller is not None else robot_config
        self._model_dir = Path(model_dir).resolve() if model_dir is not None else _model_directory()
        self._host, self._port, self._period = host, port, 1.0 / rate_hz
        self._lock = threading.Lock()
        self._latest = None
        self._stop = threading.Event()
        self._thread = None
        self._server = None
        self._closed = False

    def update(self, state, *, received_at=None):
        """Copy one SDK JointState; never render or send network data here.

        Unchanged SDK timestamps do not refresh the feedback age. received_at
        may preserve an earlier time.monotonic() receipt time for replay/bridges;
        it must use this host's monotonic clock. Joint positions are radians and
        gripper_pos is total opening in metres. Caller arrays are not retained.
        """
        q = np.asarray(state.pos(), dtype=float).copy()
        width, stamp = float(state.gripper_pos), float(state.timestamp)
        received = time.monotonic() if received_at is None else float(received_at)
        if q.shape != (self._config.joint_dof,) or not np.isfinite(q).all():
            raise ValueError("Invalid joint feedback shape or non-finite position")
        if not np.isfinite([width, stamp, received]).all():
            raise ValueError("Non-finite gripper, timestamp or receipt time")
        if received > time.monotonic():
            raise ValueError("Feedback receipt time is in the future")
        with self._lock:
            if self._latest is None or stamp != self._latest[2]:
                self._latest = (q, width, stamp, received)

    def start(self):
        """Start the viewer; returns self. Repeated starts are harmless."""
        if self._closed:
            raise RuntimeError("Create a new viewer after close()")
        if self._server is not None:
            return self
        try:
            import viser
            from viser.extras import ViserUrdf
            from scipy.spatial.transform import Rotation
        except ImportError as exc:
            raise ImportError("Install arx5-interface[visualization] to use ViserViewer") from exc

        self._robot, self._joint_names, self._finger_names = _load_model(self._config, self._model_dir)
        self._rotation = Rotation
        server = self._server = viser.ViserServer(host=self._host, port=self._port)
        try:
            server.gui.configure_theme(dark_mode=False, show_logo=False, show_share_button=False)
            server.gui.main_panel.dock_right()
            server.scene.set_up_direction("+z")
            self._root = server.scene.add_frame("/robot", show_axes=False)
            self._visual = ViserUrdf(server, self._robot, root_node_name="/robot",
                                     mesh_color_override=(0.89804, 0.91765, 0.92941))
            self._root.visible = False
            server.scene.add_grid("/grid", width=1.5, height=1.5, cell_size=0.05, plane="xy")
            server.scene.add_frame("/base", axes_length=0.10, axes_radius=0.002)
            self._eef_frame = server.scene.add_frame("/eef", axes_length=0.08, axes_radius=0.002)
            self._eef_frame.visible = False
            self._status = server.gui.add_markdown("Waiting for feedback")
            with server.gui.add_folder("Joints"):
                self._joints = server.gui.add_markdown("—")
                self._gripper = server.gui.add_markdown("Gripper: — mm")
            with server.gui.add_folder(f"EEF ({self._config.base_link_name})"):
                self._eef = server.gui.add_markdown("—")

            @server.on_client_connect
            def on_connect(client):
                client.camera.position = (0.65, -0.65, 0.45)
                client.camera.look_at = (0.08, 0, 0.15)
                client.camera.up_direction = (0, 0, 1)

            self._thread = threading.Thread(target=self._run, name="arx5-viewer", daemon=True)
            self._thread.start()
        except Exception:
            server.stop()
            self._server = None
            raise
        return self

    def _render(self, sample):
        q, width, _, _ = sample
        cfg = dict(zip(self._joint_names, q))
        for name in self._finger_names:
            limits = self._robot.joint_map[name].limit
            cfg[name] = float(np.clip(width / 2, limits.lower, limits.upper))
        with self._server.atomic():
            self._visual.update_cfg(cfg)
            transform = self._robot.get_transform(self._config.eef_link_name, self._config.base_link_name)
            rotation = self._rotation.from_matrix(transform[:3, :3])
            self._eef_frame.position = tuple(transform[:3, 3])
            self._eef_frame.wxyz = tuple(rotation.as_quat()[[3, 0, 1, 2]])
            self._root.visible = self._eef_frame.visible = True
            self._joints.content = "| Joint | rad | deg |\n|:--|--:|--:|\n" + "\n".join(
                f"| {name} | {value:+.5f} | {np.rad2deg(value):+.3f} |"
                for name, value in zip(self._joint_names, q)
            )
            self._gripper.content = f"Gripper: **{width * 1000:.2f} mm**"
            self._eef.content = "| Position | mm |\n|:--|--:|\n" + "\n".join(
                f"| {axis} | {value * 1000:+.2f} |" for axis, value in zip("XYZ", transform[:3, 3])
            ) + "\n\n| Orientation | deg |\n|:--|--:|\n" + "\n".join(
                f"| {axis} | {value:+.3f} |"
                for axis, value in zip(("Roll", "Pitch", "Yaw"), rotation.as_euler("xyz", degrees=True))
            )

    def _run(self):
        previous = None
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                if self._controller is not None:
                    self.update(self._controller.get_joint_state())
                with self._lock:
                    sample = self._latest
                if sample is not None:
                    if sample is not previous:
                        self._render(sample)
                        previous = sample
                    age = max(0.0, time.monotonic() - sample[3])
                    status = "Live" if age < 0.5 else "Delayed" if age < 2.0 else "Stopped"
                    self._status.content = f"**{status}** · {age:.2f} s"
                self._stop.wait(max(0.0, self._period - (time.monotonic() - started)))
        except Exception:
            logging.getLogger(__name__).exception("Visualization stopped; controller was not modified")
            self._status.content = "**Error**"

    def close(self):
        """Stop polling and the web server, without changing the controller."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
        if self._server is not None:
            self._server.stop()
            self._server = None
        self._controller = None
        self._closed = True

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()
