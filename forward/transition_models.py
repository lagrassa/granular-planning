import numpy as np

def is_free_space_motion(state, action):
    """

    :param state:  beginning state
    :param action: action
    :return: whether the action occurs in free space. Mostly just uses collision_fn from state
    """
    pass

def transition_model(state, action, simulator):
    """
    Given current state and action return next state
    :param state:
    :param action:
    :param simulator instance
    :return:
    """
    if collision_free_action(state, action):
        return _free_space_transition_model(state, action)
    return _with_blocks_transition_model(state, action, simulator)

def _free_space_transition_model(state, action):
    """
    Determines next state, requires that this is a free space motion
    :param state:
    :param action:
    :return:
    """
    pass

def _with_blocks_transition_model(state, action, simulator):
    """
    Using a simulator, returns the next state
    :param state:
    :param action:
    :param simulator:
    :return:
    """
    simulator.set_state(state)
    simulator.apply_action(action)
    simRobotState, simBlkStates = simulator.get_robot_blk_states()
    return simRobotState, simBlkStates

def collision_free_action(state, action):
    """
    Predicts whether there will be a collision by applying action to state
    """
    return False

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
        # assume we have 8 directions. 0 is right, 2 is up, 4 is right, 6 is down
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
