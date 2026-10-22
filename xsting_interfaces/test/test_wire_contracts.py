from rclpy.serialization import serialize_message, deserialize_message
from xsting_interfaces.msg import ArmInput, ArmTarget, TaskState
from xsting_interfaces.action import RunTask, MoveArm
from xsting_interfaces.srv import GetTarget, ReserveSlot, CommitTransfer, LoadModel
import pytest

@pytest.mark.parametrize("cls", [ArmInput, ArmTarget, TaskState, RunTask.Goal,
    RunTask.Result, RunTask.Feedback, MoveArm.Goal, MoveArm.Result, MoveArm.Feedback,
    GetTarget.Request, GetTarget.Response, ReserveSlot.Request, ReserveSlot.Response,
    CommitTransfer.Request, CommitTransfer.Response, LoadModel.Request, LoadModel.Response])
def test_generated_type_round_trip(cls):
    msg = cls()
    assert deserialize_message(serialize_message(msg), cls) == msg

def test_arm_identity_and_expiry_survive_serialization():
    msg = ArmTarget(task_id="pick-17", generation=9, input_id=41,
                    model_version="v001", joint_names=["joint_1", "joint_2"],
                    delta_positions=[0.01, -0.02])
    msg.header.frame_id = "arm_base"
    msg.header.stamp.sec = 123
    msg.expires_at.sec = 124
    out = deserialize_message(serialize_message(msg), ArmTarget)
    assert (out.task_id, out.generation, out.input_id) == ("pick-17", 9, 41)
    assert out.expires_at.sec == 124
    assert list(out.delta_positions) == [0.01, -0.02]
