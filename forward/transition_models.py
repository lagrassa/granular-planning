import numpy as np

def transition_model(state, robot_state, block_state, action,
                     threshold=1e-3, sim_flag=False, steps=2):
    """
    Given current state and action return next state
    :param state: state.State object
    :param robot_state: parent (x,y,theta) state of the robot
    :param block_state: parent Nx2 array of the (x,y) states of the objects
    :param action: tuple of the forward/backward motion and rotation to apply to robot
    :param threshold: threshold to use for collision checking in meters
    :param sim_flag: set to True to always use simulator to get succesors
    :return: 
        robot state: (x,y,theta) state of the robot as np.array
        object states: Nx2 array of the (x,y) states of the objects
    """
    if sim_flag:
        state.use_simulator=True
        is_free = 0 
    else:
        is_free = 1
        # discretize action
        action_steps = np.array(action) / steps
        
        # set parent state with free space transition model
        state.use_simulator=False
        state.set_state(robot_state, block_state)

        # iterate over discretized actions
        for i in range(steps):
            state.apply_action(action_steps)

            # check whether to use simulator for dynamics
            if not (state.is_free_space_motion(threshold=threshold)):
                is_free = 0
                break
        
        # return state if simulator is not needed
        if not state.use_simulator:
            r, b = state.get_state()
            return r, b, is_free

    # set parent state
    state.set_state(robot_state, block_state)

    # apply action
    state.apply_action(action)

    r, b = state.get_state()
    return r, b, is_free


def parseActionDTheta(action_type, stepXY, stepTheta):
    """Inside a graph, there are 4 graph actions: 
    0. move forward along heading direction
    1. move backward along heading direction
    2. roate clockwise
    3. rotate counter clockwise
    Changes to (d, theta) representation where d can be negative (indicating to move backward)
    """
    if action_type == 0:
        return (stepXY, 0)
    elif action_type == 1:
        return (-stepXY, 0)
    elif action_type == 2:
        return (0, stepTheta)
    elif action_type == 3:
        return (0, -stepTheta)
    else:
        raise ValueError("Invalid graphAction= {}".format(action_type))


def parseAction(graphAction, graphHeading, stepXY, stepTheta):
    """Inside a graph, there are 4 graph actions: 
    0. move forward along heading direction
    1. move backward along heading direction
    2. roate clockwise
    3. rotate counter clockwise
    This function parse a graph action representation to a simulator action

    :param graphAction: integer {0, 1, 2, 3}
    :param graphState: integer {0, 1, 2, 3}
    """
    diagStep = np.sqrt(2) / 2 * stepXY

    if graphAction == 0:
        #TODO(wpu): I don't know how to handle directions other than 4 or 8. So I 
        # assume we have 8 directions. 0 is right, 2 is up, 4 is left, 6 is down
        if graphHeading == 2:
            simAction = np.array([stepXY, 0, 0])
        elif graphHeading == 3:
            simAction = np.array([diagStep, -diagStep, 0])
        elif graphHeading == 0:
            simAction = np.array([0, -stepXY, 0])
        elif graphHeading == 1:
            simAction = np.array([-diagStep, -diagStep, 0])
        else:
            raise ValueError("Invalid graphHeading={}".format(graphHeading))
    elif graphAction == 1:
        if graphHeading == 2:
            simAction = np.array([-stepXY, 0, 0])
        elif graphHeading == 3:
            simAction = np.array([-diagStep, diagStep, 0])
        elif graphHeading == 0:
            simAction = np.array([0, stepXY, 0])
        elif graphHeading == 1:
            simAction = np.array([diagStep, diagStep, 0])
        else:
            raise ValueError("Invalid graphHeading= {}".format(graphAction))
    elif graphAction == 2:
        simAction = np.array([0, 0, stepTheta])
    elif graphAction == 3:
        simAction = np.array([0, 0, -stepTheta])
    else:
        raise ValueError("Invalid graphAction= {}".format(graphAction))

    return simAction
