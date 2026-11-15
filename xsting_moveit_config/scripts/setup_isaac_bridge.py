"""Run in Isaac Sim 6 Script Editor after opening xsting_manipulation.usda.

Enable isaacsim.ros2.bridge first. This authors a graph in the current stage;
press Play to start communication. Adjust ROBOT if the asset is referenced elsewhere.
"""
import omni.graph.core as og
import omni.usd
from pxr import Sdf, UsdPhysics

ROBOT='/xsting'
GRAPH='/xsting_ros2'
stage=omni.usd.get_context().get_stage()
assert stage.GetPrimAtPath(ROBOT+'/Physics/world_anchor'), 'Open the manipulation asset first'
if not stage.GetPrimAtPath('/physicsScene'):
    scene=UsdPhysics.Scene.Define(stage,'/physicsScene')
    scene.CreateGravityMagnitudeAttr(9.81)
if stage.GetPrimAtPath(GRAPH): stage.RemovePrim(GRAPH)
K=og.Controller.Keys
nodes=[('Tick','omni.graph.action.OnPlaybackTick'),
       ('Time','isaacsim.core.nodes.IsaacReadSimulationTime'),
       ('Read','isaacsim.sensors.physics.IsaacReadJointState'),
       ('Context','isaacsim.ros2.bridge.ROS2Context'),
       ('Publish','isaacsim.ros2.bridge.ROS2PublishJointState'),
       ('Subscribe','isaacsim.ros2.bridge.ROS2SubscribeJointState'),
       ('Control','isaacsim.core.nodes.IsaacArticulationController'),
       ('Clock','isaacsim.ros2.bridge.ROS2PublishClock')]
connections=[('Tick.outputs:tick','Read.inputs:execIn'),('Read.outputs:execOut','Publish.inputs:execIn'),
             ('Tick.outputs:tick','Subscribe.inputs:execIn'),('Subscribe.outputs:execOut','Control.inputs:execIn'),
             ('Tick.outputs:tick','Clock.inputs:execIn'),('Time.outputs:simulationTime','Clock.inputs:timeStamp'),
             ('Subscribe.outputs:jointNames','Control.inputs:jointNames'),
             ('Subscribe.outputs:positionCommand','Control.inputs:positionCommand')]
for field in ['jointNames','jointPositions','jointVelocities','jointEfforts','jointDofTypes','stageMetersPerUnit','sensorTime']:
    connections.append((f'Read.outputs:{field}',f'Publish.inputs:{field}'))
for node in ['Publish','Subscribe','Clock']:
    connections.append(('Context.outputs:context',node+'.inputs:context'))
og.Controller.edit({'graph_path':GRAPH,'evaluator_name':'execution'},{
    K.CREATE_NODES:nodes,K.CONNECT:connections,K.SET_VALUES:[
        ('Control.inputs:robotPath',ROBOT),('Read.inputs:prim',[Sdf.Path(ROBOT)]),
        ('Publish.inputs:topicName','/isaac_joint_states'),
        ('Subscribe.inputs:topicName','/isaac_joint_commands'),('Clock.inputs:topicName','/clock')]})
print('xSting ROS 2 bridge ready. Press Play, then launch hardware:=isaac use_sim_time:=true.')
