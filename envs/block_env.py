import os
import pybullet as p
from . import pb_utils as ut
import pybullet_data
import numpy as np
import ipdb


class Simulator:
    def __init__(self, workspace_size, goal_hole_width, gui=False, num_boxes = 2):
        if gui:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)

        self.workspace_size = workspace_size
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(1/100.)
        scaling = 10
        self.num_uses = 0
        self.box_width = 0.01*scaling
        self.height = 0.01*scaling
        #self.robot  = ut.create_box(self.pusher_length, self.pusher_width, self.height, color = (0,1,0,1), mass=0.1)
        urdf_folder = os.path.split(os.path.abspath(__file__))[0]
        self.robot = p.loadURDF(os.path.join(urdf_folder, "paddle.urdf"))
        #self.boxes  = [ut.create_box(self.box_width, self.box_width, self.height, color = (1,0,0,1), mass = 5) for _ in range(num_boxes)]
        self.boxes  = [p.loadURDF(os.path.join(urdf_folder, "cyl.urdf")) for _ in range(num_boxes)]
        self.mu = 0.1
        self.restitution = 0.1
        p.changeDynamics(self.robot, -1, restitution=self.restitution,linearDamping=1, lateralFriction=self.mu, jointDamping =0.01)
        [p.changeDynamics(box, -1, restitution=self.restitution, linearDamping = 0.99, angularDamping =0.99, lateralFriction=self.mu, jointDamping=0.01) for box in self.boxes]
        self.goal_hole_width=goal_hole_width
        ut.enable_gravity()
        self.plane_height = 0.5
        self.setup_hole()
        self.box_pose_idx = 3
        #self.cid = p.createConstraint(self.robot, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
        self.force = 5.0#0.2

    def set_motors(self, robot_pos, theta):
        maxVel = 1
        p.setJointMotorControl2(self.robot, 0, p.POSITION_CONTROL, robot_pos[0], maxVelocity=maxVel, force=self.force, targetVelocity=0) 
        p.setJointMotorControl2(self.robot, 1, p.POSITION_CONTROL, robot_pos[1], maxVelocity=maxVel, force=self.force, targetVelocity=0) 
        p.setJointMotorControl2(self.robot, 2, p.POSITION_CONTROL, theta, maxVelocity=maxVel, force=self.force, targetVelocity=0) 

    def setup_hole(self): 
        # for a workspace_size*workspace_size square
        workspace_size = self.workspace_size 
        #self.plane = p.loadURDF("plane.urdf", [0,0,0])
        plane_height = self.plane_height
        side_box_width = (workspace_size-self.goal_hole_width)/2
        left_box = ut.create_box(workspace_size, (workspace_size-self.goal_hole_width)/2,  plane_height)
        right_box = ut.create_box(workspace_size, (workspace_size-self.goal_hole_width)/2,  plane_height)
        top_box = ut.create_box((workspace_size-self.goal_hole_width)/2,self.goal_hole_width, plane_height)
        bottom_box = ut.create_box((workspace_size-self.goal_hole_width)/2,self.goal_hole_width, plane_height)
        
        center_distance_sides = 0.5*(side_box_width+self.goal_hole_width)
        ut.set_point(left_box, (0, center_distance_sides, -plane_height/2))
        ut.set_point(right_box, (0, -center_distance_sides, -plane_height/2))
        ut.set_point(top_box, (center_distance_sides,0, -plane_height/2))
        ut.set_point(bottom_box, (-center_distance_sides,0, -plane_height/2))
 
        blocks = [left_box, right_box, top_box, bottom_box]
        [p.changeDynamics(block, -1, restitution=self.restitution, lateralFriction=self.mu) for block in blocks]

    def set_state(self, state, debug=True):
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
            # TODO: update this condition with a more accurate one
            if np.max(np.abs(box_pose)) < self.goal_hole_width / 2:
                new_h = self.height*0.5 - self.plane_height * (i + 1)
            else:
                new_h = self.height*0.5
            new_pos = np.hstack([box_pose, new_h])
            ut.set_pose(self.boxes[i], (new_pos, (1,0,0,0)) )
        self.set_motors(robot_pos, robot_orn)
        if debug:
            assert np.allclose(state, self.get_state())
            orn = ut.get_pose(self.robot)[1]
            euler = ut.euler_from_quat(orn)
            if (euler[0] or euler[1] or euler[2]):
                ipdb.set_trace()
            
        #p.changeConstraint(self.cid, robot_pos, quat, maxForce=100)
    def close(self):
        p.disconnect()
    def set_robot_blk_states(self, rState, bStates):
        """Set simulator internal states
        :param rState: 1D np array (x, y, theta)
        :param bStates: 2D np array [(x, y)]
        """
        state = np.concat((rState, bStates.flat))
        return self.set_state(state)


    def apply_action(self, action, show_error=False):
        """
        Applies action to environment and then simulates it
        (d, theta), but only one value can be nonzero
        :param action:
        :return:
        """
        #tune these parameters to make the physics easy
        self.num_uses +=1 
        assert (not action[0] or not action[1])
        delta_yaw = action[1]
        cur_q = ut.get_joint_positions(self.robot, (0,1,2))
        cur_theta = cur_q[2]
        des_theta = cur_theta+delta_yaw
        ref = np.pi/2.
        delta_x = action[0]*np.cos(ref+cur_theta)
        delta_y = action[0]*np.sin(ref+cur_theta)
        des_pos = (cur_q[0]+delta_x, cur_q[1]+delta_y)
        #des_quat = ut.quat_from_euler([0,0,cur_theta+delta_yaw])
        #p.changeConstraint(self.cid, des_pos, des_quat, maxForce=self.force)
        duration = 1
        self.set_motors(des_pos, des_theta)
        ut.simulate_for_duration(duration)
        if show_error:
            cur_q = ut.get_joint_positions(self.robot, (0,1,2))
            print("pos error:", np.linalg.norm(np.array(cur_q[:2])-np.array(des_pos)))
            print("angle error:", np.linalg.norm(cur_q[2]-des_theta))

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

    def get_robot_blk_states(self):
        """Returns the robot and block states. robot state is (x, y, theta), block state is (x, y)
        :return: (1D np.array with 3 elements, 2D np.array number-of-blocks x 2)
        """
        state = self.get_state()
        robotState = state[:3]
        boxState = state[3:].reshape([len(self.boxes), 2])   
        return robotState, boxState  


if __name__ == "__main__":
    world = Simulator(1.5, 0.3, gui=True, num_boxes = 2)
    robot_state  = [0.0,-0.37,0.05]
    box_states  = [0.2,0.2,0,0.3]
    state = np.hstack([robot_state,box_states])
    test_state = [ 0.        ,  0.3       ,  2.35619449,  0.2       , -0.1       ,
        0.        ,  0.        ]
    world.set_state(test_state)
    import ipdb; ipdb.set_trace()
    obs = world.get_state()
    assert(np.allclose(state, obs))
    world.apply_action([0,0.1])
    world.apply_action([0.1, 0])
    world.apply_action([-0.1,0])


    robot_state = [0, 0.5, 0.0]
    box_states = [0.6, 0, 0, 0.6]
    state = np.hstack([robot_state,box_states])
    world.set_state(state)
    import ipdb; ipdb.set_trace()
    world.apply_action([0,0.1])


