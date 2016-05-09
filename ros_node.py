#!/usr/bin/env python

import rospy

from enterprise_interface import *
from lao_star import lao_star
from generate_mdp import mdp
import time
import random
import pdb

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

vision_ct = 0

def chall_num(chall):
    if chall == 'vision':
        vision_ct += 1
        return str(vision_ct)
    return '1'


def predicates_to_state(preds):
    challenges = [
                    "mctschallenge1",
                    "reachabilitychallenge1",
                    "visionchallenge1",
                    "visionchallenge2",
                    "visionchallenge3"               
                 ]

    challenge_tuple = ['0']*len(challenges)

    print preds
    for p in preds:
        if p[:10] == "(at rover ":
            loc = p[10:-1]
        for i,c in enumerate(challenges):
            if p == "(challenge_complete "+c+")":
                challenge_tuple[i] = '1'

    return (loc_map.get(loc,loc),tuple(challenge_tuple))


def stochastic_action(action_map, noise=0): #noise 0-1 (+/- percent noise)
    action_prob_tuples = [[action_map[k],k] for k in action_map]
    #add noise to probabilities
    action_prob_tuples = [[n[0]*(1+(noise*random.random()*2-noise)), n[1]] for n in action_prob_tuples]
    prob_sum = sum([n[0] for n in action_prob_tuples])
    # normalize
    action_prob_tuples = [[n[0]/prob_sum,n[1]] for n in action_prob_tuples]
    action_prob_tuples = sorted(action_prob_tuples)
    #choose action from dist
    rand = random.random()
    for prob,act in action_prob_tuples:
        rand-=prob
        if rand <= 0.0001: return act


def action_to_pddl(action,currstate):
    if action == "do challenge":
        num = 1
        if chall_map[currstate[0]] == "vision":
            vision_chall = list(currstate[1][2:])
            index = vision_chall.index('0')
            num = index+1

        return ("(do_challenge_"+chall_map[currstate[0]]+" rover "+map_loc[currstate[0]]+" "+chall_map[currstate[0]]+"challenge" + str(num) + ")")
    actions = mdp.T(currstate, action)

    '''next1 = actions.keys()[0]
    next2 = actions.keys()[1]
    best_action = None
    if actions[next1] > actions[next2]:
        best_action = next1
    else:
        best_action = next2'''

    best_action = stochastic_action(actions, noise=0)

    return "(move rover "+map_loc[currstate[0]]+" "+map_loc[best_action[0]]+")"



if __name__ == '__main__':
    # Connect to ROS
    rospy.init_node('enterprise_interface_example')

    # Create an instance of the class
    ei = EnterpriseInterface()

    # get policy
    pi = lao_star(mdp)
    print "found policy"
    #pdb.set_trace()
    #print pi

    pred_state = ei.get_predicates()

    while not pred_state:
        time.sleep(1.0)
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

