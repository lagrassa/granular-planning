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
    pass

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
    return simulator.get_state()
