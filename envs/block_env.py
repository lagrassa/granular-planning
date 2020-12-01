import pybullet as p
import pb_utils as ut
import pybullet_data
import numpy as np
class Simulator:
    def __init__(self, gui=False, num_boxes = 2):
        if gui:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        scaling = 10
        self.pusher_length = 0.1*scaling
        self.pusher_width = 0.01*scaling
        self.box_width = 0.01*scaling
        self.height = 0.01*scaling
        #self.robot  = ut.create_box(self.pusher_length, self.pusher_width, self.height, color = (0,1,0,1), mass=0.1)
        self.robot = p.loadURDF("paddle.urdf")
        #self.boxes  = [ut.create_box(self.box_width, self.box_width, self.height, color = (1,0,0,1), mass = 5) for _ in range(num_boxes)]
        self.boxes  = [p.loadURDF("cyl.urdf") for _ in range(num_boxes)]
        p.changeDynamics(self.robot, -1, restitution=0.02,linearDamping=1, lateralFriction=0.99, jointDamping =0.01)
        [p.changeDynamics(box, -1, restitution=0.02, linearDamping = 0.99, angularDamping =0.99, lateralFriction=0.99, jointDamping=0.01) for box in self.boxes]
        self.plane = p.loadURDF("plane.urdf", [0,0,0])
        p.changeDynamics(self.plane, -1, restitution=0.98, lateralFriction=0.99)
        self.box_pose_idx = 3
        #self.cid = p.createConstraint(self.robot, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
        self.force = 1#0.2
    def set_motors(self, robot_pos, theta):
        maxVel = 1
        p.setJointMotorControl2(self.robot, 0, p.POSITION_CONTROL, robot_pos[0], maxVelocity=maxVel, force=self.force, targetVelocity=0) 
        p.setJointMotorControl2(self.robot, 1, p.POSITION_CONTROL, robot_pos[1], maxVelocity=maxVel, force=self.force, targetVelocity=0) 
        p.setJointMotorControl2(self.robot, 2, p.POSITION_CONTROL, theta, maxVelocity=maxVel, force=self.force, targetVelocity=0) 
        

    def set_state(self, state):
        """
        Given planner state, sets the simulator state to reflect that by teleporting
        :param state:
        :return:
        """
        #unpack state
        robot_pos = tuple(state[:2]) #x,y,theta
        robot_orn = state[2]
        for i in range(3):
            ut.set_joint_position(self.robot,i, state[i])
        for i, box in enumerate(self.boxes):
            box_pose = state[self.box_pose_idx + 2*i:self.box_pose_idx+2*i+2]
            ut.set_point(self.boxes[i], np.hstack([box_pose, self.height*0.5]))
        self.set_motors(robot_pos, robot_orn)
        #p.changeConstraint(self.cid, robot_pos, quat, maxForce=100)


    def apply_action(self, action):
        """
        Applies action to environment and then simulates it
        :param action:
        :return:
        """
        #tune these parameters to make the physics easy
        delta_yaw = action[2]
        cur_q = ut.get_joint_positions(self.robot, (0,1,2))
        des_pos = (cur_q[0]+action[0], cur_q[1]+action[1])
        cur_theta = cur_q[2]
        #des_quat = ut.quat_from_euler([0,0,cur_theta+delta_yaw])
        des_theta = cur_theta+delta_yaw
        #p.changeConstraint(self.cid, des_pos, des_quat, maxForce=self.force)
        duration = 1 
        self.set_motors(des_pos, des_theta)
        ut.simulate_for_duration(duration)

    def get_state(self):
        """
        Returns the current state
        :param state:
        :return:
        """
        state = np.zeros(3+(2*len(self.boxes)))
        state[:3] = ut.get_joint_positions(self.robot, (0,1,2))
        for i, box in enumerate(self.boxes):
            pose = ut.get_pose(box)
            state[self.box_pose_idx+2*i:self.box_pose_idx+2*i+2] = pose[0][:2]
        return state
        

if __name__ == "__main__":
    world = Simulator(gui=True, num_boxes = 2)
    robot_state  = [0,0,0.05]
    box_states  = [0.2,0.2,0,0.3]
    state = np.hstack([robot_state,box_states])
    world.set_state(state)
    obs = world.get_state()
    assert(np.allclose(state, obs))
    shift_y = np.array([0,0.05,0])
    world.apply_action(shift_y)
    world.apply_action(shift_y)
    world.apply_action(shift_y)
    world.apply_action(shift_y)
    print("Test passed")

