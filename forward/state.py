import numpy as np

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

        if robot_dims is not None:
            self._robot_dims = robot_dims
        else:
            # TODO check that this is width, height not height width
            self._robot_dims = np.array([0.8, 0.1])
        if obj_dims is not None:
            self._obj_dims = obj_dims
        else:
            # TODO update this whenever we make changes to blocks
            self._obj_dims = 0.05

        # assert self._obj_states.shape[0] == self._obj_dims.shape[0]
        
        # TODO need to update this everytime you add an object, assuming state is defined at obj/robot center
        self.check_distance = np.max(self._robot_dims) / 2 + np.max(self._obj_dims) / 2

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
        if self.use_simulator:
            return self.sim.get_robot_blk_states()
        else:
            return self._robot_state, self._obj_states

    def apply_action(self, action):
        """
        Move robot using a forward/backward motion and rotation.
        :param action: size 2 tuple of the (d, theta) movement to apply to the robot
        """
        if self.use_simulator:
            self.sim.apply_action(action)
        else:
            translation = action[0]
            delta_theta = action[1]
            rob_state = self.get_robot_state()
            theta = rob_state[-1]
            delta_x = translation * np.cos(theta)
            delta_y = translation * np.sin(theta)
            self._robot_state += np.array([delta_x, delta_y, delta_theta]).reshape(3)

    def set_obj_state(self, obj_id, x, y):
        """
        Set the state of a single object in the simulator
        :param obj_id: int that represents the array index of the desired object
        :param x: the x position of the object as a float
        :param y: the y position of the object as a float
        """
        if self.use_simulator:
            robot_state, obj_states = self.sim.get_robot_blk_states()
            obj_states[obj_id,:] = np.array([x,y])
            self.sim.set_robot_blk_states(robot_state, obj_states)
        else:
            self._obj_states[obj_id, :] = np.array([x, y])

    def set_obj_states(self, poses):
        """
        Set the states of all the objects in the simulator       
        :param poses: (Nx2) np.array of the [(x, y)] poses of the objects
        """
        if self.use_simulator:
            robot_state, obj_states = self.sim.get_robot_blk_states()
            obj_states = poses
            self.sim.set_robot_blk_states(robot_state, obj_states)
        else:
            self._obj_states = poses

    def set_robot_state(self, x, y, theta):
        """
        Set (x,y) position and orientation of robot
        :param x: the desired x position of the robot as a float
        :param y: the desired y position of the robot as a float
        :param theta: the desired orientation of the object as a float
        """
        if self.use_simulator:
            robot_state, obj_states = self.sim.get_robot_blk_states()
            robot_state = np.array([x,y,theta])
            self.sim.set_robot_blk_states(robot_state, obj_states)
        else:
            self._robot_state = np.array([x, y, theta]).reshape(3)

    def set_state(self, robot_state, obj_states):
        """
        Set the states of the robot and objects
        :param robot_state: np.array of the (x,y,theta) state of the robot
        :param poses: (Nx2) np.array of the [(x, y)] poses of the objects
        """
        if self.use_simulator:
            state = np.concatenate((robot_state, obj_states.flat))
            self.sim.set_state(state)
        else:
            self._robot_state = robot_state
            self._obj_states = obj_states

    def get_robot_bbox(self):
        """
        Return the four corners of the robot bounding box in CC order
        Ref: https://stackoverflow.com/a/56848101
        """
        x, y, theta = self.get_robot_state()
        center = np.array([x,y])
        w, h = self._robot_dims

        v1 = np.array([np.cos(theta), np.sin(theta)])
        v2 = np.array([-v1[1], v1[0]]) # rotate by 90 degrees

        v1 *= w / 2
        v2 *= h / 2

        return np.array([center + v1 + v2,
                         center - v1 + v2,
                         center - v1 - v2,
                         center + v1 - v2])

    def is_free_space_motion(self, threshold=1e-2):
        """
        Return True if the robot is close to being in collision with an object.
        Ref: https://hackmd.io/@US4ofdv7Sq2GRdxti381_A/ryFmIZrsl?type=view 
        """
        # Get all the objects that could possibly be in collision with robot
        nearest_objs, nearest_dist = self.get_nearest_blocks(threshold=threshold)
        
        if nearest_objs.shape[0] == 0:
            # return False if no objects are close to robot
            return False
        # elif nearest_dist[0] <= np.min(self._robot_dims):
        #     return True
        else:
            # check if any of the objects near robot are in collision
            in_collision = True # Assuming robot is in collision unless proven that it is not
            for i in range(nearest_objs.shape[0]):
                obj_id = nearest_objs[i]

                # TODO change the way to get vertices below when shapes aren't circles
                # TODO can get bbox around circle that is aligned with robot if this is too slow
                obj_vertices = get_circle_bbox(self._obj_states[obj_id], self._obj_dims)
                edges = get_edges(obj_vertices)
                
                robot_vertices = self.get_robot_bbox()
                edges += get_edges(robot_vertices)

                normals = [orthog_vec(edge) for edge in edges]

                for normal in normals:
                    if(is_seperated(normal, robot_vertices, obj_vertices, threshold=threshold)):
                        in_collision = False
                        break
                
                if in_collision:
                    self.use_simulator = True
                    return True
                else:
                    in_collision = True

            self.use_simulator = False
            return False


    def get_nearest_blocks(self, threshold=1e-2):
        """
        Return the object ID's of the objects within possible collision distance to the robot.
        """
        dist = np.linalg.norm(self._robot_state[:-1].reshape(1,2) - self._obj_states, axis=0)
        sorted_obj_ids = np.argsort(dist)
        idx = np.argmax(dist[sorted_obj_ids] >= (self.check_distance-threshold))
        obj_ids = sorted_obj_ids[:idx]
        return obj_ids, dist[obj_ids]


def get_edges(vertices):
    """
    Return the vectors for the edges of a polygon
    """
    edges = []
    for i in range(len(vertices)):
        edge = vertices[(i+1)%len(vertices)] - vertices[i]
        edges.append(edge)
    return edges

def orthog_vec(v):
    """
    Return a 90 degree clockwise rotation of a 2-D vector
    """
    return np.array([-v[1], v[0]])

def is_seperated(normal, vertices1, vertices2, threshold=1e-2):
    """
    Return True if two polygons are separated along an axis that is orthogonal to a polygon edge.
    Uses SAT to check if the projections of the polygons are in collision along the given normal vector.

    :param normal: 2-D np.array (x,y) that represents a vector normal to the edge you are checking along.
    :param vertices1: Nx2 np.array of the (x,y) positions of the vertices of first polygon in CC order.
    :param vertices1: Nx2 np.array of the (x,y) positions of the vertices of second polygon in CC order.
    :param threshold: a float of how much tolerance to give between the polygons to consider it in collision.
    """
    min1 = np.inf
    min2 = np.inf
    max1 = -np.inf
    max2 = -np.inf

    for v in vertices1:
        proj = np.dot(v, normal)

        min1 = min(min1, proj)
        max1 = max(max1, proj)

    for v in vertices2:
        proj = np.dot(v, normal)

        min2 = min(min2, proj)
        max2 = max(max2, proj)

    if (max1 - min2 >= -threshold) and (max2 - min1 >= -threshold):
        return False
    else:
        return True

def get_circle_bbox(position, radius):
    """
    Return the vertices of a x-axis/y-axis aligned bounding around a circle
    """
    return np.array([position + np.array([radius, radius]),
                     position + np.array([-radius, radius]),
                     position + np.array([-radius, -radius]),
                     position + np.array([radius, -radius])]
    )

