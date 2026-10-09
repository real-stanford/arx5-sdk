"""Hardware-free regression checks: python -m pytest python/tests/test_visualization.py.

Requires pytest plus requirements_visualization.txt. Uses loopback HTTP servers,
fake read-only controllers, and repository models; never opens CAN or writes files.
"""

import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yourdfpy

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("visualization", ROOT / "python" / "visualization.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ViserViewer = module.ViserViewer


@pytest.fixture
def config():
    return SimpleNamespace(robot_model="X5", urdf_path=str(ROOT / "models" / "X5.urdf"),
                           joint_dof=6, base_link_name="base_link", eef_link_name="eef_link")


def state(q=None, width=0.04, timestamp=1.0):
    q = np.zeros(6) if q is None else q
    return SimpleNamespace(pos=lambda: q, gripper_pos=width, timestamp=timestamp)


def wait_for(predicate):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Viewer did not reach the expected state")


def test_gripper_only_preserves_runtime_fk(config):
    runtime = Path(config.urdf_path)
    original_bytes = runtime.read_bytes()

    def mesh_path(fname):
        path = ROOT / "models" / fname
        if not path.is_file() and Path(fname).name == "base_link.STL":
            path = ROOT / "models" / "meshes" / "X5" / "base_link.STL"
        return str(path)

    original = yourdfpy.URDF.load(str(runtime), filename_handler=mesh_path)
    display, names, fingers = module._load_model(config, ROOT / "models")
    assert names == [f"joint{i}" for i in range(1, 7)]
    assert fingers == ["joint7", "joint8"]
    for name in original.link_map:
        left, right = original.link_map[name].inertial, display.link_map[name].inertial
        if left is not None:
            assert left.mass == right.mass
            np.testing.assert_array_equal(left.inertia, right.inertia)
            np.testing.assert_array_equal(left.origin, right.origin)
    for q in (np.zeros(6), np.array([.1, .4, .8, -.3, .2, -.1])):
        original.update_cfg(q)
        for width in (0, .044, .088):
            display.update_cfg(dict(zip(names + fingers, list(q) + [width / 2] * 2)))
            for name in original.link_map:
                np.testing.assert_array_equal(display.get_transform(name, "base_link"),
                                              original.get_transform(name, "base_link"))
            left = display.get_transform("link7", "link6")
            right = display.get_transform("link8", "link6")
            assert left[1, 3] - right[1, 3] == pytest.approx(.024896 + .0249 + width)
    assert runtime.read_bytes() == original_bytes


def test_update_copies_and_does_not_refresh_frozen_feedback(config):
    viewer = ViserViewer(robot_config=config)
    q = np.zeros(6)
    old_time = time.monotonic() - 10
    viewer.update(state(q), received_at=old_time)
    q[:] = 1
    viewer.update(state(q))  # Same SDK timestamp: not fresh feedback.
    assert np.all(viewer._latest[0] == 0)
    assert viewer._latest[3] == old_time
    viewer.update(state(q, timestamp=0))  # A restarted producer may reset its clock.
    assert np.all(viewer._latest[0] == 1)
    viewer.close()


@pytest.mark.parametrize("sample", [state(np.zeros(5)), state(np.full(6, np.nan)),
                                     state(width=np.inf), state(timestamp=np.nan)])
def test_reject_invalid_feedback(config, sample):
    viewer = ViserViewer(robot_config=config)
    with pytest.raises(ValueError):
        viewer.update(sample)


def test_manual_status_and_gripper_rendering(config):
    with ViserViewer(robot_config=config, port=0) as viewer:
        assert viewer.start() is viewer
        assert viewer._status.content == "Waiting for feedback"
        for stamp, age, expected in ((1, 0, "Live"), (2, .7, "Delayed"), (3, 3, "Stopped")):
            viewer.update(state(width=-.0005, timestamp=stamp), received_at=time.monotonic() - age)
            wait_for(lambda: expected in viewer._status.content)
        assert "-0.50 mm" in viewer._gripper.content  # Preserve raw feedback.
        assert viewer._robot.cfg[-2:].tolist() == [0, 0]  # Only visuals are clipped.
        assert viewer._root.visible
    assert not viewer._thread.is_alive()
    viewer.close()
    with pytest.raises(RuntimeError):
        viewer.start()


def test_auto_poll_is_read_only_and_close_leaves_controller_alive(config):
    class Controller:
        reads = 0

        def get_robot_config(self):
            return config

        def get_joint_state(self):
            self.reads += 1
            return state(timestamp=self.reads)

        def __getattr__(self, name):
            raise AssertionError(f"Unexpected controller access: {name}")

    controller = Controller()
    with ViserViewer(controller, port=0) as viewer:
        wait_for(lambda: controller.reads >= 3 and "Live" in viewer._status.content)
    reads = controller.reads
    time.sleep(.1)
    assert controller.reads == reads
    assert controller.get_joint_state().timestamp == reads + 1


def test_poll_error_is_visible_without_controller_cleanup(config, caplog):
    class Controller:
        def get_robot_config(self):
            return config

        def get_joint_state(self):
            raise RuntimeError("feedback unavailable")

    with ViserViewer(Controller(), port=0) as viewer:
        wait_for(lambda: viewer._status.content == "**Error**")
    assert "feedback unavailable" in caplog.text
