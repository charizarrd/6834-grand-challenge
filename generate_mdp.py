import itertools
from mdp import *

actions = ["up", "down", "left", "right", "do challenge"]
locations = ["loc" + str(i) for i in range(1,10)]

locations_to_map = {}
for l in locations:
    locations_to_map[l] = l
locations_to_map["loc1"] = "station_3"
locations_to_map["loc2"] = "home"
locations_to_map["loc5"] = "corridor"
locations_to_map["loc6"] = "recharging_terminal"
locations_to_map["loc7"] = "station_2"
locations_to_map["loc9"] = "station_1"

# 0 represents challenge not done, 1 otherwise
# challenge 1 - 5
init = ("loc2", tuple(list("000")))
terminals = []
for l in locations:
    terminals.append((l, tuple(list("111"))))

# maps challenge number (1-5) to location?
challenge_locations = {}
challenge_locations[1] = "loc9"
challenge_locations[2] = "loc7"
challenge_locations[3] = "loc1"

# get all possible challenge states
challenge_states = [tuple(list("000")), tuple(list("111"))]
for i in itertools.permutations(list("100")):
    if i not in challenge_states:
        challenge_states.append(tuple(list(i)))

for i in itertools.permutations(list("110")):
    if i not in challenge_states:
        challenge_states.append(tuple(list(i)))

# for i in itertools.permutations(list("11100")):
# 	if i not in states:
# 		challenge_states.append(list(i))

# for i in itertools.permutations(list("11110")):
# 	if i not in states:
# 		challenge_states.append(list(i))

all_states = []
for loc in locations:
    for c in challenge_states:
        all_states.append((loc, c))

# MDP TRANSITIONS
transitions = {}
prob_succeed_challenge = 0.8
prob_fail_challenge = 1.0 - prob_succeed_challenge

# challenge location transitions
for num, loc in challenge_locations.iteritems():
    #print "num", num, "loc", loc
    # for every challenge state that has a 0 at index num-1, transition to 1 at index num-1
    # i.e. if challenge 1, every state with 0xxxx --> 1xxxx
    # assuming we don't allow robot to do the challenge again if it has already succeeded?
    for s in challenge_states:
        if s[num-1] == "0":
            current_state = (loc, s)
            new_state = list(s)[:]
            new_state[num-1] = "1"
            new_state = (loc, tuple(new_state))
            # state, action, new state, prob
            key = (current_state, "do challenge")
            if key not in transitions:
                transitions[key] = {}

            transitions[key][new_state] = prob_succeed_challenge
            transitions[key][current_state] = prob_fail_challenge
            #print current_state, new_state, "do challenge"
            # print current_state do challenge, current_state, prob of failing challenge

# moving transitions
prob_action_succeed = 0.8
prob_action_fail = 1.0 - prob_action_succeed

# action up
for i in [1, 4, 5, 3]:
    for c in challenge_states:
        current_state = ("loc" + str(i), c)
        new_state = ("loc" + str(i+3), c)

        key = (current_state, "up")
        if key not in transitions:
            transitions[key] = {}

        transitions[key][new_state] = prob_action_succeed
        transitions[key][current_state] = prob_action_fail
        #print current_state, new_state, "up"

# action down
for i in [7, 8, 4, 6]:
    for c in challenge_states:
        current_state = ("loc" + str(i), c)
        new_state = ("loc" + str(i-3), c)

        key = (current_state, "down")
        if key not in transitions:
            transitions[key] = {}

        transitions[key][new_state] = prob_action_succeed
        transitions[key][current_state] = prob_action_fail


# action right
for i in [4, 5, 8, 2]:
    for c in challenge_states:
        current_state = ("loc" + str(i), c)
        new_state = ("loc" + str(i+1), c)
        
        key = (current_state, "right")
        if key not in transitions:
            transitions[key] = {}
            
        transitions[key][new_state] = prob_action_succeed
        transitions[key][current_state] = prob_action_fail


# action left
for i in [9, 5, 6, 3]:
    for c in challenge_states:
        current_state = ("loc" + str(i), c)
        new_state = ("loc" + str(i-1), c)

        key = (current_state, "left")
        if key not in transitions:
            transitions[key] = {}

        transitions[key][new_state] = prob_action_succeed
        transitions[key][current_state] = prob_action_fail


# MDP REWARDS??
rewards={}
for state in all_states:
    if state is not ('loc9', ('1', '1', '1')):
        if state not in rewards:
            rewards[state] = {}
        rewards[state] = 0
rewards[('loc9', ('1', '1', '1'))] = 100
rewards[('loc7', ('1', '1', '1'))] = 100
rewards[('loc1', ('1', '1', '1'))] = 100

#mdp.terminals = ('loc9', ('1', '1', '1'))
        
#print rewards
#rewards = {('loc9', ('1', '1', '1')): 100}
# {'station_2': 1.0, 'station_3': -0.4, 'station_4': -0.4, 'station_5': -0.4, 'station_7': -0.4, 'station_8': -0.4, 'station_9': -0.4}

gamma = 0.9
mdp = MDP(all_states, actions, init, rewards, transitions, terminals, gamma)
#print all_states
#print actions
#print init
#s1  (('0', '0', '0'), ('1', '0', '0')) p  0.8 s  ('loc7', ('0', '0', '0')) a  do challenge