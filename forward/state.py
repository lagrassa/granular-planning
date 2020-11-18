import numpy as np

class EnvRepresentation:
    def __init__(self, robot_state, robot_dims, obj_states, obj_dims):
        """
        :param robot_state: initial robot state. 1x3 numpy array of the values: (x, y, theta).
        :param obj_states: inital states of the objects in the environment. Nx2 numpy array
            of the (x, y) position of each object center.
        :param obj_dims: Nx1 array that decribes the dimensions of the corresponding objects
            in obj_states.
        """
        assert obj_states.shape[0] == obj_dims.shape[0]
        self._robot_state = robot_state
        self._robot_dims = robot_dims
        self._obj_states = obj_states
        self._obj_dims = obj_dims

    def add_cylinder(self, radius, x_init, y_init):
        """
        :param radius: The radius of the cylinder to be added.
        :param x_init: The initial x position of the cylinder center.
        :param y_init: The initial y position of the cylinder center.
        """
        self._obj_states = np.append(self.obj_states, [[x_init,y_init]])
        self._obj_dims = np.append(self.obj_dims, [radius])

    def add_block(self, ):
        raise NotImplementedError

    def get_obj_state(self, obj_id):
        #TODO probably just switch this with pybullet code
        return self._obj_states[id, :]

    def get_obj_dim(self, obj_id):
        return self._obj_dim[id]

    def get_robot_state(self):
        return self._robot_state

    def move_obj(self, obj_id, delta_x, delta_y):
        #TODO probably just switch this with pybullet code
        self._obj_states[id, :] += np.array([delta_x, delta_y])

    def teleport_obj(self, obj_id, x, y):
        #TODO probably just switch this with pybullet code
        self._obj_states[id, :] = np.array([x, y])

    def move_robot(self, delta_x, delta_y, delta_theta):
        self._robot_state += np.array([delta_x, delta_y, delta_theta]).reshape(1,3)

    def teleport_robot(self, x, y, theta):
        self._robot_state = np.array([x, y, theta]).reshape(1,3)

    def get_robot_bbox(self):
        """
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

    def is_robot_in_collision(self, threshold=1e-3, obj_ids=None):
        """

        """
        raise NotImplementedError
        # for i in range(obj_states.shape[0]):

    def is_seperated():
        """
        Return True if the two polygons are separated
        """
        raise NotImplementedError
