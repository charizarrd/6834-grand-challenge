#!/usr/bin/env python

import rospy

from enterprise_interface import *
from lao_star import *
from generate_mdp import mdp
import time

loc_map = {"station_3":"loc1",
           "home":"loc2",
           "corridor":"loc5",
           "recharging_terminal":"loc6",
           "station_2":"loc7",
           "station_1":"loc9",
           "loc3":"loc3",
           "loc4":"loc4",
           "loc8":"loc8"}
map_loc = {loc_map[k]:k for k in loc_map}

chall_map = {
    "loc9":"mcts",
    "loc7":"reachability",
    "loc1":"vision"
    }

def predicates_to_state(preds):
    challenge_tuple = ['0','0','0']
    if "(challenge_complete mctschallenge1)" in preds:
        challenge_tuple[0] = '1'
    if "(challenge_complete reachabilitychallenge1)" in preds:
        challenge_tuple[1] = '1'
    if "(challenge_complete visionchallenge1)" in preds:
        challenge_tuple[2] = '1'
    for p in preds:
        if p[:10] == "(at rover ":
            loc = p[10:-1]

    return (loc_map.get(loc,loc),tuple(challenge_tuple))

def action_to_pddl(action,currstate):
    if action == "do challenge":
        return ("(do_challenge_"+chall_map[currstate[0]]+" rover "+map_loc[currstate[0]]+" "+chall_map[currstate[0]]+"challenge1)")
    actions = mdp.T(currstate, action)

    next1 = actions.keys()[0]
    next2 = actions.keys()[1]
    best_action = None
    if actions[next1] > actions[next2]:
        best_action = next1
    else:
        best_action = next2

    return "(move rover "+map_loc[currstate[0]]+" "+map_loc[best_action[0]]+")"



if __name__ == '__main__':
    # Connect to ROS
    rospy.init_node('enterprise_interface_example')

    # Create an instance of the class
    ei = EnterpriseInterface()

    # get policy
    pi = lao_star(mdp)

    #print pi

    pred_state = ei.get_predicates()
    state = predicates_to_state(pred_state)

    print pred_state
    print state
    while state not in mdp.terminals:
        action = pi[state]
        pddl_action = action_to_pddl(action,state)
        print state
        print pddl_action
        ei.dispatch_activity(pddl_action)

        time.sleep(0.5)

        pred_state = ei.get_predicates()
        state = predicates_to_state(pred_state)

    print "done!"
