from mdp import *

def get_reachable_states(s, mdp, policy, reachable_states = None):
	"""
	s: start state
	mdp: MDP object
	policy: dictionary mapping state to action
	reachable_states: None or list of reachable states

	returns: list of reachable states from state s given policy
	"""
	if not reachable_states:
		reachable_states = [s]
	current = s
	if s in policy:
		action = policy[s]
		for (result_state, prob) in mdp.T(current, action).iteritems():
			if result_state not in reachable_states and prob > 0.0:
				reachable_states.append(result_state)
				get_reachable_states(result_state, mdp, policy, reachable_states)
	return reachable_states

def get_children(s, mdp):
    """
    s: current_state
    mdp: MDP objects
    
    returns: list of direct children states from state s
    """
    children = []

    for a in mdp.actions(s):
        for (result_state, prob) in mdp.T(s, a).iteritems():
            if result_state not in children and prob > 0.0:
                children.append(result_state)

    return children


def simple_heuristic(s, mdp):
    discount_factor = mdp.gamma
    best_goal_cost = 0
    for s in mdp.terminals:
        if mdp.R(s) > best_goal_cost:
            best_goal_cost = mdp.R(s)
    return best_goal_cost/(1-discount_factor)


def lao_star(mdp):
    """
    mdp: MDP problem to solve
    
    returns: dictionary mapping expanded states to policies
    """
    s_envelope = [mdp.init]
    s_terminal = [mdp.init]
    pi = None
    
    while True:
        r_envelope = {}
        for s in s_envelope:
            if s in s_terminal:
                r_envelope[s] = simple_heuristic(s, mdp)
            else:
                r_envelope[s] = mdp.R(s)
        
        t_envelope = {}
        for s in s_envelope:
            for a in mdp.actlist:
                if s in s_terminal:
                    t_envelope[(s,a)] = {}
                else:
                    t_envelope[(s,a)] = mdp.T(s, a)
        
        partial_mdp = MDP(s_envelope, mdp.actlist, mdp.init, r_envelope, t_envelope, mdp.terminals, mdp.gamma)
                
        # find optimal policy on states in envelope
        pi = policy_iteration(partial_mdp)
        
        # find reachable states
        reachable_states = []
        reachable_states = get_reachable_states(mdp.init, mdp, pi)
        reachable_states = list(set(reachable_states).intersection(s_terminal))
        
        # get children states of reachable states
        reachable_children = []
        for s in reachable_states:
            reachable_children += get_children(s, mdp)
        
        # define new terminal states
        new_terminals = []
        
        # add current terminal states if not in reachable states
        for s in s_terminal:
            if s not in reachable_states:
                new_terminals.append(s)
        
        # add children if not in envelope
        for s in reachable_children:
            if s not in s_envelope:
                new_terminals.append(s)
        
        # add reachable children to envelope
        for s in reachable_children:
            if s not in s_envelope:
                s_envelope.append(s)
                
        # check if intersection between terminal states and reachable states is empty
        if set(s_terminal).isdisjoint(reachable_states):
            break
        else:
            s_terminal = new_terminals
    
    return pi
