import gtpyhop

domain_name = 'd3d_assembly_cell'
domain = gtpyhop.Domain(domain_name=domain_name)

rigid = gtpyhop.State('rigid relations')
# These types are used by the 'is_a' helper function, later in this file
rigid.types = {
    'part':   ['part1', 'part2'],
    'robot': ['r11', 'r12', 'r13', 'r14'],
    'handler_robot': ['r11', 'r13'],
    'uv_robot': ['r12', 'r14'],
    'adh_robot': ['r12'],
    'location': [
        'table',
        'adh_loc',
        'join_loc',
        'r11_ee',
        'r12_ee',
        'r13_ee',
        'r14_ee'
        ]
    }

rigid.robot_data = {
    'r11': {
        'ee_loc': 'r11_ee'
    },
    'r13': {
        'ee_loc': 'r13_ee'
    }
}

def is_a(var, type):
    return var in rigid.types[type]

# Actions (state to state)
def pick_part(state, part, robot):
    if (is_a(part, 'part') and is_a(robot, 'robot')):
        part_loc = state.part_loc[part]
        robot_loc = state.robot_loc[robot]
        robot_holding = state.robot_holding[robot]
        if ((part_loc == robot_loc) and ('' == robot_holding)):
            state.part_loc[part] = rigid.robot_data[robot]['ee_loc']
            state.robot_holding[robot] = part
            return state

def move_robot(state, robot, loc_curr, loc_dst):
    if  (not (is_a(robot, 'robot')
        and is_a(loc_curr, 'location')
        and is_a(loc_dst, 'location')
        )):
        return
    robot_loc = state.robot_loc[robot]
    if (robot_loc != loc_dst):
        state.robot_loc[robot] = loc_dst
        return state


def join(state, partA, robotA, partB, robotB):
    if  (not (is_a(partA, 'part')
        and is_a(robotA, 'robot')
        and is_a(partB, 'part')
        and is_a(robotB, 'robot'))):
        return

    partA_loc = state.part_loc[partA]
    partB_loc = state.part_loc[partB]
    robotA_loc = state.robot_loc[robotA]
    robotB_loc= state.robot_loc[robotB]
    if (not (partA_loc == rigid.robot_data[robotA]['ee_loc']
        and partB_loc == rigid.robot_data[robotB]['ee_loc']
        and robotA_loc == 'join_loc'
        and robotB_loc == 'join_loc')):
        return
    state.part_conn[partA] = partB
    state.part_conn[partB] = partA
    return state

gtpyhop.declare_actions(pick_part, move_robot, join)

# Methods
def move_robot_do_nothing(state, robot, loc):
    if (is_a(robot, 'robot') and is_a(loc, 'location')):
        if (state.robot_loc[robot] == loc):
            return []
def move_robot_actual(state, robot, loc):
    if (is_a(robot, 'robot') and is_a(loc, 'location')):
        if (state.robot_loc[robot] != loc):
            return [('move_robot', robot, state.robot_loc[robot], loc)]
gtpyhop.declare_task_methods('move_robot_t', move_robot_do_nothing, move_robot_actual)

def assemble_parts(state, part1, part2, robot1, robot2):
    parts = [part1, part2]
    robots = [robot1, robot2]
    if (
        all(map(lambda x: is_a(x, 'part'), parts))
        and (all(map(lambda x: is_a(x, 'robot'), robots)))
        and (robot1 != robot2)
        and (part1 != part2)
        ):
        return [
            ('move_robot_t', robots[0], state.part_loc[parts[0]]),
            ('move_robot_t', robots[1], state.part_loc[parts[1]]),
            ('pick_part', parts[0], robots[0]),
            ('pick_part', parts[1], robots[1]),
            ('move_robot_t', robots[0], 'join_loc'),
            ('move_robot_t', robots[1], 'join_loc'),
            ('join', parts[0], robots[0], parts[1], robots[1])
            ]

def ground(f, **kwargs):
    num_params = len(kwargs.keys())
    types_list = []
    for (param_name, param_type) in kwargs.items():
        types_list.append(rigid.types[param_type])

    funcs = []

    import itertools
    import copy
    for combo in itertools.product(*types_list):
        print("Combo Iters: " + str(combo))
        def wrapped(*argss):
            total_args = argss + combo
            print("Combos Wrapped: " + str(combo))
            return f(*total_args)
        funcs.append(wrapped)
    return funcs

gtpyhop.declare_task_methods(
    'assemble',
    *ground(assemble_parts, r1='handler_robot', r2='handler_robot')
    )

gtpyhop.current_domain = domain

state0 = gtpyhop.State('state0')
state0.part_loc = {
    'part1': 'table',
    'part2': 'table'
}
state0.robot_loc = {
    'r11': 'join_loc',
    'r13': 'table'
}
state0.robot_holding = {
    'r11': '',
    'r13': ''
}
state0.part_conn= {
}

state1 = state0.copy()

gtpyhop.verbose = 3
res = gtpyhop.find_plan(state1, [('assemble', 'part1', 'part2')])