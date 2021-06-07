"""
Examples file for blocks_tasks.
-- Dana Nau <nau@umd.edu>, June 6, 2021
"""

# For use in debugging:
# from IPython import embed
# from IPython.terminal.debugger import set_trace

import gtpyhop

# Code for use in paging and debugging
from test_harness import check_result, pause, set_trace


#############     beginning of tests     ################

def run_examples():

    gtpyhop.print_domain()

    print("\nLet's call find_plan on some simple things that should fail.\n")

    state1 = gtpyhop.State('state1')
    state1.pos={'a':'b', 'b':'table', 'c':'table'}
    state1.clear={'c':True, 'b':False,'a':True}
    state1.holding={'hand':False}

    state1.display('Initial state is')

    plan = gtpyhop.find_plan(state1,[('pickup','a')])
    check_result(plan,False)

    plan = gtpyhop.find_plan(state1,[('pickup','b')])
    check_result(plan,False)

    plan = gtpyhop.find_plan(state1,[('get','b')])
    check_result(plan,False)

    pause()
    print("""
Next, some simple things that should succeed. As a reminder, in state1,
block a is on block b, block b is on the table, and block c is on the table.
""")
    
    plan = gtpyhop.find_plan(state1,[('pickup','c')])
    check_result(plan, [('pickup','c')])

    plan = gtpyhop.find_plan(state1,[('get','a')])
    check_result(plan, [('unstack','a', 'b')])

    plan = gtpyhop.find_plan(state1,[('get','c')])
    check_result(plan, [('pickup','c')])

    plan = gtpyhop.find_plan(state1,[('move_one','a','table')])
    check_result(plan, [('unstack','a', 'b'), ('putdown','a')])
    pause()


    print("""
A Multigoal is a data structure that specifies desired values for some of the
state variables. Below, goal1a says that we want the following configuration:
    c on b,   b on a,   a on the table
""")

    state1.display("Initial state is")

    goal1a = gtpyhop.Multigoal('goal1a')
    goal1a.pos={'c':'b', 'b':'a', 'a':'table'}

    goal1a.display()
    
    print("""
'move_blocks' is the task of moving the blocks until they are in the desired
positions. The initial state says a, b, and c are the only blocks. Thus
('move_blocks', goal1a) and ('move_blocks', goal1b) have the same solutions.

First, here's ('move_blocks', goal1a):
    """)

    # Tell the test harness what answer to expect, so it can signal an error
    # if gtpyhop returns an incorrect answer. Checks like this have been very
    # very helpful for debugging both gtpyhop and the various example domains.

    expected = [('unstack', 'a', 'b'), ('putdown', 'a'), ('pickup', 'b'), ('stack', 'b', 'a'), ('pickup', 'c'), ('stack', 'c', 'b')] 

    plan1 = gtpyhop.find_plan(state1,[('move_blocks', goal1a)])
    check_result(plan1,expected)

    print("""
Like goal1a, goal1b says we want c on b on a. It omits "a on table", 
but as you'll see on the next page, it will still produce the same plan. 
That's because the only way to get c on b on a is if a is on the table.
""")

    goal1b = gtpyhop.Multigoal('goal1b')
    goal1b.pos={'c':'b', 'b':'a'}

    goal1b.display()

    pause()
    
    gtpyhop.verbose = 2
    plan2 = gtpyhop.find_plan(state1,[('move_blocks', goal1b)])
    check_result(plan2,expected)
    pause()

    print("""
Another initial state and two multigoals, goal2a and goal2b, such that
('move_blocks', goal2a) and ('move_blocks', goal2b) have the same solutions.
""")
    
    state2 = gtpyhop.State('state2')
    state2.pos={'a':'c', 'b':'d', 'c':'table', 'd':'table'}
    state2.clear={'a':True, 'c':False,'b':True, 'd':False}
    state2.holding={'hand':False}

    state2.display('Initial state is')
    
    goal2a = gtpyhop.Multigoal('goal2a')
    goal2a.pos={'b':'c', 'a':'d', 'c':'table', 'd':'table'}
    goal2a.clear={'a':True, 'c':False,'b':True, 'd':False}
    goal2a.holding={'hand':False}

    goal2a.display()
    
    goal2b = gtpyhop.Multigoal('goal2b')
    goal2b.pos={'b':'c', 'a':'d'}

    goal2b.display()
    
    ### goal2b omits some of the conditions of goal2a,
    ### but those conditions will need to be achieved anyway.

    expected = [('unstack', 'a', 'c'), ('putdown', 'a'), ('unstack', 'b', 'd'), ('stack', 'b', 'c'), ('pickup', 'a'), ('stack', 'a', 'd')] 

#     pause()
#     print("When we run find_plan on the tasks ('move_blocks', goal2a) and")
#     print("('move_blocks', goal2b), it produces the same plan for both:")


    gtpyhop.verbose = 1
    plan1 = gtpyhop.find_plan(state2,[('move_blocks', goal2a)])
    check_result(plan1,expected)

    plan2 = gtpyhop.find_plan(state2,[('move_blocks', goal2b)])
    check_result(plan2,expected)
    pause()


    print("\nRun find_plan on problem bw_large_d from the SHOP distribution:\n")

    state3 = gtpyhop.State('state3')
    state3.pos = {1:12, 12:13, 13:'table', 11:10, 10:5, 5:4, 4:14, 14:15, 15:'table', 9:8, 8:7, 7:6, 6:'table', 19:18, 18:17, 17:16, 16:3, 3:2, 2:'table'}
    state3.clear = {x:False for x in range(1,20)}
    state3.clear.update({1:True, 11:True, 9:True, 19:True})
    state3.holding={'hand':False}

    state3.display('Initial state is')
    
    goal3 = gtpyhop.Multigoal('goal3')
    goal3.pos = {15:13, 13:8, 8:9, 9:4, 4:'table', 12:2, 2:3, 3:16, 16:11, 11:7, 7:6, 6:'table'}
    goal3.clear = {17:True, 15:True, 12:True}

    goal3.display()
    
    expected = [('unstack', 1, 12), ('putdown', 1), ('unstack', 19, 18), ('putdown', 19), ('unstack', 18, 17), ('putdown', 18), ('unstack', 17, 16), ('putdown', 17), ('unstack', 9, 8), ('putdown', 9), ('unstack', 8, 7), ('putdown', 8), ('unstack', 11, 10), ('stack', 11, 7), ('unstack', 10, 5), ('putdown', 10), ('unstack', 5, 4), ('putdown', 5), ('unstack', 4, 14), ('putdown', 4), ('pickup', 9), ('stack', 9, 4), ('pickup', 8), ('stack', 8, 9), ('unstack', 14, 15), ('putdown', 14), ('unstack', 16, 3), ('stack', 16, 11), ('unstack', 3, 2), ('stack', 3, 16), ('pickup', 2), ('stack', 2, 3), ('unstack', 12, 13), ('stack', 12, 2), ('pickup', 13), ('stack', 13, 8), ('pickup', 15), ('stack', 15, 13)] 

    plan = gtpyhop.find_plan(state3,[('move_blocks', goal3)])
    check_result(plan,expected)
    pause()

    print("\nRun find_plan on problem BW-rand-50 from the IPC-2011 distribution.\n")

     
    print("- Define initial state for problem IPC2011BWrand50:")
     
    IPC2011BWrand50 = gtpyhop.State('problem BW-rand-50')
    IPC2011BWrand50.pos = {    
            1:48, 2:33, 3:41, 4:37, 5:45, 6:16, 7:31, 8:28, 9:49,
            10:34, 11:15, 12:17, 13:20, 14:2, 15:44, 16:5, 17:32, 18:50, 19:30,
            20:22, 21:27, 22:38, 23:11, 24:'table', 25:46, 26:'table', 27:40, 28:43, 29:19,
            30:39, 31:29, 32:'table', 33:'table', 34:14, 35:36, 36:'table', 37:8, 38:9, 39:18,
            40:3, 41:35, 42:4, 43:24, 44:26, 45:47, 46:42, 47:1, 48:21, 49:25,
            50:6
            }
    IPC2011BWrand50.clear = {x:False for x in range(1,50)}
    IPC2011BWrand50.clear.update({7:True, 10:True, 12:True, 13:True, 23:True})
    IPC2011BWrand50.holding = {'hand':False}
     
    IPC2011BWrand50.display('Initial state is')

     
    IPC2011BWrand50Goal = gtpyhop.Multigoal('problem BW-rand-50')
    IPC2011BWrand50Goal.pos = {1:33, 3:40, 4:46, 5:21, 6:17, 7:37, 8:15, 9:41,  
            10:26, 11:23, 12:25, 13:47, 14:20, 15:19, 16:31, 17:39, 18:50, 19:1,
            20:45, 21:11, 23:43, 25:42, 26:36, 27:35, 28:29, 29:44,
            30:8, 31:9, 32:6, 33:10, 34:14, 35:2, 36:7, 37:32, 38:28,
            40:24, 41:38, 42:34, 43:12, 44:49, 45:4, 46:18, 47:30, 48:22,
            50:13}
    IPC2011BWrand50Goal.clear = {}
     
    IPC2011BWrand50Goal.display()

    expected = [('unstack', 7, 31), ('putdown', 7), ('unstack', 10, 34), ('putdown', 10), ('unstack', 12, 17), ('putdown', 12), ('unstack', 13, 20), ('putdown', 13), ('unstack', 17, 32), ('putdown', 17), ('unstack', 20, 22), ('putdown', 20), ('unstack', 22, 38), ('putdown', 22), ('unstack', 23, 11), ('putdown', 23), ('unstack', 11, 15), ('putdown', 11), ('unstack', 15, 44), ('putdown', 15), ('unstack', 31, 29), ('putdown', 31), ('unstack', 29, 19), ('putdown', 29), ('unstack', 19, 30), ('putdown', 19), ('unstack', 30, 39), ('putdown', 30), ('unstack', 39, 18), ('putdown', 39), ('pickup', 17), ('stack', 17, 39), ('unstack', 18, 50), ('putdown', 18), ('unstack', 34, 14), ('putdown', 34), ('unstack', 14, 2), ('putdown', 14), ('unstack', 2, 33), ('putdown', 2), ('unstack', 38, 9), ('putdown', 38), ('unstack', 9, 49), ('putdown', 9), ('unstack', 49, 25), ('putdown', 49), ('unstack', 44, 26), ('stack', 44, 49), ('pickup', 29), ('stack', 29, 44), ('unstack', 25, 46), ('putdown', 25), ('unstack', 46, 42), ('putdown', 46), ('unstack', 42, 4), ('putdown', 42), ('unstack', 4, 37), ('putdown', 4), ('unstack', 37, 8), ('putdown', 37), ('unstack', 8, 28), ('putdown', 8), ('unstack', 28, 43), ('stack', 28, 29), ('pickup', 38), ('stack', 38, 28), ('unstack', 43, 24), ('putdown', 43), ('unstack', 50, 6), ('putdown', 50), ('unstack', 6, 16), ('stack', 6, 17), ('pickup', 32), ('stack', 32, 6), ('pickup', 37), ('stack', 37, 32), ('pickup', 7), ('stack', 7, 37), ('unstack', 16, 5), ('putdown', 16), ('unstack', 5, 45), ('putdown', 5), ('unstack', 45, 47), ('putdown', 45), ('unstack', 47, 1), ('putdown', 47), ('unstack', 1, 48), ('putdown', 1), ('unstack', 48, 21), ('stack', 48, 22), ('unstack', 21, 27), ('putdown', 21), ('unstack', 27, 40), ('putdown', 27), ('unstack', 40, 3), ('stack', 40, 24), ('unstack', 3, 41), ('stack', 3, 40), ('unstack', 41, 35), ('stack', 41, 38), ('pickup', 9), ('stack', 9, 41), ('pickup', 31), ('stack', 31, 9), ('pickup', 16), ('stack', 16, 31), ('unstack', 35, 36), ('stack', 35, 2), ('pickup', 27), ('stack', 27, 35), ('pickup', 36), ('stack', 36, 7), ('pickup', 26), ('stack', 26, 36), ('pickup', 10), ('stack', 10, 26), ('pickup', 33), ('stack', 33, 10), ('pickup', 1), ('stack', 1, 33), ('pickup', 19), ('stack', 19, 1), ('pickup', 15), ('stack', 15, 19), ('pickup', 8), ('stack', 8, 15), ('pickup', 30), ('stack', 30, 8), ('pickup', 47), ('stack', 47, 30), ('pickup', 13), ('stack', 13, 47), ('pickup', 50), ('stack', 50, 13), ('pickup', 18), ('stack', 18, 50), ('pickup', 46), ('stack', 46, 18), ('pickup', 4), ('stack', 4, 46), ('pickup', 45), ('stack', 45, 4), ('pickup', 20), ('stack', 20, 45), ('pickup', 14), ('stack', 14, 20), ('pickup', 34), ('stack', 34, 14), ('pickup', 42), ('stack', 42, 34), ('pickup', 25), ('stack', 25, 42), ('pickup', 12), ('stack', 12, 25), ('pickup', 43), ('stack', 43, 12), ('pickup', 23), ('stack', 23, 43), ('pickup', 11), ('stack', 11, 23), ('pickup', 21), ('stack', 21, 11), ('pickup', 5), ('stack', 5, 21)] 
    
    plan = gtpyhop.find_plan(IPC2011BWrand50,[('move_blocks', IPC2011BWrand50Goal)])
    check_result(plan,expected)
    pause()

    print("""
Call run_lazy_lookahead on the following problem, with verbose=1:
""")

    state2.display(heading='Initial state is')
    goal2b.display(heading='Goal is')

    new_state = gtpyhop.run_lazy_lookahead(state2, [('move_blocks', goal2b)])

    pause()

    print("The goal should now be satisfied, so the planner should return an empty plan:\n")

    plan = gtpyhop.find_plan(new_state, [('move_blocks', goal2b)])
    check_result(plan,[])

    print("No more examples")

###############################################################################
# It's tempting to put a call to main() at this point, to run the examples
# without making the user type an extra command. But if we do this and an
# error occurs during execution of main(), we get a situation in which the
# actions and methods files have been imported but the current file hasn't
# been -- which causes problems if we try to import the current file again.
###############################################################################

if __name__=="__main__":
    main()
