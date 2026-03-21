
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
modelId = p.loadURDF("model/single_pendulum.urdf", startPos, startOrientation)
p.resetDebugVisualizerCamera(
    cameraDistance=8, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.5])
for joint_id in range(p.getNumJoints(modelId)):
    p.setJointMotorControl2(modelId,
                            joint_id,
                            p.VELOCITY_CONTROL,
                            force=0)

stickId = 0
pendulumId = 1

while True:
    keys = p.getKeyboardEvents()

    torque = 0.0
    if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        torque = 300.0
    if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        torque = -300.0

    p.setJointMotorControl2(bodyUniqueId=modelId,
                            jointIndex=0,
                            controlMode=p.TORQUE_CONTROL,
                            force=torque)

    stickPos, stickVel, _, _ = p.getJointState(modelId, stickId)
    pendulumPos, pendulumVel, _, _ = p.getJointState(modelId, pendulumId)

    print(stickPos,stickVel,pendulumPos,pendulumVel)

    # Step simulation
    p.stepSimulation()
    time.sleep(1./240.)
    """
while True:
    p.stepSimulation()
    time.sleep(1./240.)
    """
