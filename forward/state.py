import numpy as np
from shapely.geometry import Point, box, Polygon
from shapely.affinity import rotate

class State:
    def __init__(self, simulator, robot_dims=None, obj_dims=None):
        """
        :param simulator: envs.block_env.Simulator object of the Pybullet simulator.
        :param robot_dims: robot dimensions. 1x2 numpy array of the values: (x_dim, y_dim).
        :param obj_dims: Float that decribes the radius of the cylinders in simulator.
        """
        self.sim = simulator
        self.use_simulator = False

        self._robot_state, self._obj_states = self.sim.get_robot_blk_states()

        self._ref_rot = np.pi/2 # phase shift so robot moves along it's local y instead of x

        if robot_dims is not None:
            self._robot_dims = robot_dims
        else:
            # TODO check that this is width, height not height width
            self._robot_dims = np.array([0.8, 0.1])
        self._robot_xdim = self._robot_dims[0]
        self._robot_ydim = self._robot_dims[1]
        if obj_dims is not None:
            self._obj_dims = obj_dims
        else:
            # TODO update this whenever we make changes to blocks
            self._obj_dims = 0.05

        # assert self._obj_states.shape[0] == self._obj_dims.shape[0]
        
        # TODO need to update this everytime you add an object, assuming state is defined at obj/robot center
        self.check_distance = np.max(self._robot_dims) / 2 + np.max(self._obj_dims)

    def get_obj_state(self, obj_id):
        """
        Get the (x,y) position of the given object
        :param obj_id: int that represents the array index of the desired object
        """
        if self.use_simulator:
            _, obj_states = self.sim.get_robot_blk_states()
            return obj_states[obj_id, :]
        else:
            return self._obj_states[obj_id, :]

    def get_obj_states(self):
        """
        Get the (Nx2) (x,y) positions of all the objects, where N is the number of objects
        """
        if self.use_simulator:
            _, obj_states = self.sim.get_robot_blk_states()
            return obj_states
        else:
            return self._obj_states

    def get_obj_dims(self):
        """
        Get the dimensions of the objects
        - TODO update this if we add different shapes
        """
        if self.use_simulator:
            return self._obj_dims
        else:
            return self._obj_dims

    def get_robot_state(self):
        """
        Get the (x,y,theta) state of the robot
        """
        if self.use_simulator:
            robot_state, _ = self.sim.get_robot_blk_states()
            return robot_state
        else:
            return self._robot_state

    def get_state(self):
        """
        Get the (x,y,theta) state of the robot and the Nx2 array of the (x,y) states of the objects
        """
        # rsim, bsim = self.sim.get_robot_blk_states()
        # rstate, bstate = self._robot_state, self._obj_states
        
        # if not np.array_equal(rstate, rsim) or not np.array_equal(bstate, bsim.all):
        #     import ipdb; ipdb.set_trace()
        
        if self.use_simulator:
            return self.sim.get_robot_blk_states()
        else:
            return self._robot_state, self._obj_states

        # robot_state, block_states = self.sim.get_robot_blk_states()
        # return robot_state, block_states

    def apply_action(self, action):
        """
        Move robot using a forward/backward motion and rotation.
        :param action: size 2 tuple of the (d, theta) movement to apply to the robot
        """
        if self.use_simulator:
            self.sim.apply_action(action)
        else:
            #TODO fix the tracking of sim vs non sim
            rob_state = self.get_robot_state()
            theta = rob_state[2]
            delta_x = action[0] * np.cos(self._ref_rot + theta)
            delta_y = action[0] * np.sin(self._ref_rot + theta)
            self._robot_state += np.array([delta_x, delta_y, action[1]]).reshape(3)

#TODO having the sim and free space both set defeats the purpose of the freespace motion model
    def set_obj_state(self, obj_id, x, y):
        """
        Set the state of a single object in the simulator
        :param obj_id: int that represents the array index of the desired object
        :param x: the x position of the object as a float
        :param y: the y position of the object as a float
        """
        # set simulator transition model state           
        robot_state, obj_states = self.sim.get_robot_blk_states()
        obj_states[obj_id,:] = np.array([x,y])
        self.sim.set_robot_blk_states(robot_state, obj_states)
        # set free space motion transition model state                      
        self._obj_states[obj_id, :] = np.array([x, y])

    def set_obj_states(self, poses):
        """
        Set the states of all the objects in the simulator       
        :param poses: (Nx2) np.array of the [(x, y)] poses of the objects
        """
         # set simulator transition model state           
        robot_state, obj_states = self.sim.get_robot_blk_states()
        obj_states = poses.copy()
        self.sim.set_robot_blk_states(robot_state, obj_states)
         # set free space motion transition model state           
        self._obj_states = poses.copy()

    def set_robot_state(self, x, y, theta):
        """
        Set (x,y) position and orientation of robot
        :param x: the desired x position of the robot as a float
        :param y: the desired y position of the robot as a float
        :param theta: the desired orientation of the object as a float
        """
         # set simulator transition model state        
        robot_state, obj_states = self.sim.get_robot_blk_states()
        robot_state = np.array([x,y,theta])
        self.sim.set_robot_blk_states(robot_state, obj_states)
        # set free space motion transition model state
        self._robot_state = np.array([x, y, theta]).reshape(3)

    def set_state(self, robot_state, obj_states):
        """
        Set the states of the robot and objects
        :param robot_state: np.array of the (x,y,theta) state of the robot
        :param poses: (Nx2) np.array of the [(x, y)] poses of the objects
        """
        # set simulator transition model state
        state = np.concatenate((robot_state, obj_states.flat))
        self.sim.set_state(state)
        # set free space motion transition model state
        self._robot_state = robot_state.copy()
        self._obj_states = obj_states.copy()

    def get_robot_bbox(self, thresh=1e-3):
        """
        Returns a shapely polygon of the robot bounding box.
        Ref: https://stackoverflow.com/a/56848101
        """
        x, y, theta = self.get_robot_state()
        center = np.array([x,y])

        v1 = np.array([np.cos(theta), np.sin(theta)])
        v2 = np.array([-v1[1], v1[0]]) # rotate by 90 degrees

        v1 *= self._robot_xdim / 2
        v2 *= self._robot_ydim / 2
        return Polygon([tuple(center + v1 + v2 + [thresh, thresh]),
                        tuple(center - v1 + v2 + [-thresh, thresh]),
                        tuple(center - v1 - v2 + [-thresh, -thresh]),
                        tuple(center + v1 - v2 + [thresh, -thresh])])

    def is_free_space_motion(self, threshold=1e-3):
        """
        Return True if the robot is close to being in collision with an object.
        :param threshold: threshold to use for collision checking in meters
        """
        robot_bbox = self.get_robot_bbox(thresh=threshold)
        for i in range(self._obj_states.shape[0]):
            temp_state = self._obj_states[i]
            # TODO change the way to get collision shape below when shapes aren't circles
            temp_point = Point(temp_state[0],temp_state[1])
            temp_circle = temp_point.buffer(self._obj_dims)

            if robot_bbox.intersects(temp_circle):
                self.use_simulator = True
                return False

        self.use_simulator = False
        return True

    def get_nearest_blocks(self, threshold=1e-2):
        """
        Return the object ID's of the objects within possible collision distance to the robot.
        #TODO I think there is a bug with this (Steven)
        """
        dist = np.linalg.norm(self._robot_state[:-1].reshape(1,2) - self._obj_states, axis=0)
        sorted_obj_ids = np.argsort(dist)
        idx = np.argmax(dist[sorted_obj_ids] >= (self.check_distance+threshold))
        obj_ids = sorted_obj_ids[:idx]
        return obj_ids, dist[obj_ids]

