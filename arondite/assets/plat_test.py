import time

import pybullet as p

if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)
    res = p.loadURDF("platform.urdf", physicsClientId=physicsClient, useFixedBase=1)
    # txtr = p.loadTexture("padmat.png")
    # p.changeVisualShape(res, 2, textureUniqueId=txtr)
    for joint in range(p.getNumJoints(res)):
        p.setJointMotorControl2(res, joint, p.VELOCITY_CONTROL, force=0)
    p.setGravity(0, 0, -9.81)
    for i in range(1000):
        if i < 20:
            p.applyExternalForce(0, 1, (0.0, 0.0, 100.0), (0.0, 0.2, 0.0), p.LINK_FRAME)
            p.applyExternalForce(0, 2, (0.0, 0.0, 100.0), (0.2, 0.0, 0.0), p.LINK_FRAME)
        # p.applyExternalForce(0, 1, (0.0, 0.0, -10000.0), (0.0, 0.0, 0.0), p.LINK_FRAME)
        p.stepSimulation()
        time.sleep(1.0 / 60.0)
