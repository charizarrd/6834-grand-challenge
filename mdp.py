"""
From Artificial Intelligence: A Modern Approach
"""

import random
import operator
import pdb
import time
from grid_mdp import *

def argmin(seq, fn):
    """Return an element with lowest fn(seq[i]) score; tie goes to first one.
    >>> argmin(['one', 'to', 'three'], len)
    'to'
    """
    best = seq[0]; best_score = fn(best)
    for x in seq:
        x_score = fn(x)
        if x_score < best_score:
            best, best_score = x, x_score
    return best

def argmax(seq, fn):
    """Return an element with highest fn(seq[i]) score; tie goes to first one.
    >>> argmax(['one', 'to', 'three'], len)
    'three'
    """
    return argmin(seq, lambda x: -fn(x))

#______________________________________________________________________________

class MDP:
    def __init__(self, states, actions, init, reward, transitions, terminals, gamma, obstacles = [], grid_size = None):
        self.states = states            # a list of strings
        self.reward = reward            # a dictionary: states -> reward
        self.transitions = transitions  # a dictionary: (state, action) -> dictionary: state-> prob
        self.terminals = terminals      # a list of strings
        self.gamma = gamma              # a float
        self.actlist = actions          # a list of strings
        self.grid_size = grid_size
        self.obstacles = obstacles
        self.init = init

    def R(self, state):
        "Return a numeric reward for this state."
        return self.reward.get(state, 0.0)

    def T(self, state, action):
        """Transition model.  From a state and an action, return a dictionary
        of result-state --> probability"""
        return self.transitions.get((state, action), {})

    def actions(self, state):
        """Set of actions that can be performed in this state.  By default, a
        fixed list of actions, except for terminal states. Override this
        method if you need to specialize by state."""
        if state in self.terminals:
            return [None]
        else:
            return self.actlist

    def to_grid(self):
        if self.grid_size:
            cols = self.grid_size[0]
            rows = self.grid_size[1]
            grid = GridMDP(cols, rows)
    
            for i in range(cols):
                for j in range(rows):
                    s = '(' + str(i) + ',' + str(j)+ ')'
                    if s in self.obstacles:
                        grid.mark_cell_as(i, j, CELL_OBSTACLE)
                    elif s in self.terminals:
                        grid.mark_cell_as(i, j, CELL_GOAL)
                    elif s == self.init:
                        grid.mark_cell_as(i, j, CELL_START)
    
                    grid.set_cell_reward(i, j, self.R(s))
    
            return grid
        else:
            print "No grid size specified"

def read_mdp(fname, gamma = 0.9):
    init = None
    reward = {}
    states = set()
    transitions = {}
    terminals = []
    actions = set()
    grid_size = None
    obstacles = []
    def update(s, a, s1, p):
        """ Update a transition entry """
        states.add(s)
        actions.add(a)
        dist = transitions.get((s, a), {})
        if s1 in dist:
            # print 'Summing', s, a, s1
            dist[s1] = dist[s1] + float(p)
        else:
            dist[s1] = float(p)
        transitions[(s,a)] = dist        
    for line in open(fname, 'r'):
        if not line: continue
        vals = line.split()
        if len(vals) == 3 and vals[0] == "grid":
            grid_size = (int(vals[1]), int(vals[2]))
        elif len(vals) == 2 and vals[0] == "obstacle":
            obstacles.append(vals[1])
        elif len(vals) == 1:              # initial state
            init = vals[0]
        elif len(vals) == 2:            # reward
            states.add(vals[0])
            reward[vals[0]] = float(vals[1])
        elif len(vals) == 3:            # reward for terminal state
            states.add(vals[0])
            reward[vals[0]] = float(vals[1])
            terminals.append(vals[0])
        elif len(vals) == 4:            # single transition
            update(*vals)
        elif len(vals) > 4:             # multiple transitions
            s = vals[0]
            a = vals[1]
            for i in range(2, len(vals), 2):
                update(s, a, vals[i], vals[i+1])

    if not init: raise Exception, 'No init state specified'

    # Normalize transition distributions that need it.
    updates = []
    for sa, dist in transitions.items():
        p = sum([pi for pi in dist.values()])
        if p == 0: raise Exception, 'Zero probability transition'
        if abs(p - 1.0) > 0.001:
            print 'Normalizing distribution for', sa
            updates.append((sa, dict([(si, pi/p) for (si, pi) in dist.items()])))
    for (sa, dist) in updates:
        transitions[sa] = dist

    print 'States=', len(states), 'Actions=', len(actions), 'Terminals', len(terminals)
        
    return MDP(list(states), list(actions), init, reward, transitions, terminals, gamma, obstacles, grid_size)


#______________________________________________________________________________


def expected_utility(a, s, U, mdp):
    "The expected utility of doing a in state s, according to the MDP and U."
    return sum([p*U[s1]*mdp.gamma for (s1,p) in mdp.T(s,a).iteritems()])


def policy_iteration(mdp):
    """
    Solve an MDP by policy iteration [Fig. 17.7]
    
    mdp: an MDP object
    
    returns: best policy, dictionary mapping state to action
    """    
    
    U = dict([(s, 0) for s in mdp.states])
    pi = dict([(s, random.choice(mdp.actions(s))) for s in mdp.states])
    
    while True:
        U = policy_evaluation(pi, U, mdp)
        unchanged = True
        for s in mdp.states:
            a = argmax(mdp.actions(s), lambda a: expected_utility(a,s,U,mdp))
            if a != pi[s]:
                pi[s] = a
                unchanged = False
        if unchanged:
            return pi

def policy_evaluation(pi, U, mdp, k=20):
    """Return an updated utility mapping U from each state in the MDP to its
    utility, using an approximation (modified policy iteration)."""
    for i in range(k):
        for s in mdp.states:
            U[s] = mdp.R(s) + expected_utility(pi[s], s, U, mdp)
    return U
