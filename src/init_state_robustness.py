#!/usr/bin/python3
  
from model import *
import numpy as np
import random
from z3 import *
import time

# Planning problem as DTS
N = 10 # Grid size, No. of states = N*N

#nObs = 40 # No. of obstacles
S = ['s_'+str(i)+'_'+str(j) for i in range(N) for j in range(N)]
A = ['U', 'D', 'L', 'R']


obs = []
for s in S:
    _,i,j = s.split('_')
    if int(i) > int(j)+4 or int(j) > int(i)+4:
        obs.append(s)

#start_states = ['s_0_'+str(i) for i in range(0,int(N/2))]
start_states = []
goals = ['s_9_6', 's_9_7', 's_9_8']
start_set = ['s_0_0',  's_0_1', 's_0_2']

# Generating labels
L = {}
for s in S:
    _,i,j = s.split('_')
    L[s] = []
    if s in start_set:
        L[s].append('Ss')
    if s in obs:
        L[s].append('X')
    if s in start_states:
        L[s].append('S')
    if s in goals:
        L[s].append('G')
    #L[s].append('O'+str(i))

# Transition function
def Tran(s,a):
    _,i,j = s.split('_')
    i = int(i)
    j = int(j)
    if a == 'D':
        i_ = i-1
        j_ = j
    if a =='U':
        i_ = i+1
        j_ = j
    if a == 'R':
        i_ = i
        j_ = j+1
    if a == 'L':
        i_ = i
        j_ = j-1
    s_ = 's_'+str(i_)+'_'+str(j_)
    if s_ in S and s_ not in obs and s not in obs:
        return s_
    else:
        return []

# Generate CDTS from planning
cdts = CDTS(S, A, Tran, L)
#cdts.plot()

# DTS from CDTS
dts = DTS(cdts)

# Bound on path length
T = 15 

# Mapping states and labels to numbers
stateToNum = {}
numToState = {}
states = dts.getStates()
n = len(states)
i = 0
for s in states:
    stateToNum[s] = i
    numToState[i] = s
    i += 1
labelToNum = {}
numToLabel = {}
labels = []
dts_labels = dts.getLabels()
for s in states:
    for l in dts_labels[s]:
        if l not in labels:
            labels.append(l)
i = 0
for l in labels:
    labelToNum[l] = i
    numToLabel[i] = l
    i += 1
trans = dts.getTrans()

# Generating z3 variables
# Path 1 - state and label variables and constraints
S1 = IntVector("s1", T)
l1 = [[Bool("l1_%i_%i"%(t,labelToNum[l])) for l in labels] for t in range(T)]
# Path 2 - state and label variables and constraints
S2 = IntVector("s2", T)
l2 = [[Bool("l2_%i_%i"%(t,labelToNum[l])) for l in labels] for t in range(T)]

# z3 solver
solver = Solver()

solver.push()
solver.add(
        And(
            [S1[t] >= 0 for t in range(T)] + [S1[t] < n for t in range(T)] # Legal states in path1
            +[Implies(S1[t] == stateToNum[s], Or([S1[t+1] == stateToNum[x] for x in trans[s]])) for t in range(T-1) for s in trans.keys()] # Path1 transitions
            +[Implies(S1[t] == stateToNum[s], And([l1[t][labelToNum[l]] if l in dts_labels[s] else Not(l1[t][labelToNum[l]]) for l in labels])) for t in range(T) for s in states] # Path1 state to labels
            + [S2[t] >= 0 for t in range(T)] + [S2[t] < n for t in range(T)] # Legal states in path2
            +[Implies(S2[t] == stateToNum[s], Or([S2[t+1] == stateToNum[x] for x in trans[s]])) for t in range(T-1) for s in trans.keys()] # Path2 transitions
            +[Implies(S2[t] == stateToNum[s], And([l2[t][labelToNum[l]] if l in dts_labels[s] else Not(l2[t][labelToNum[l]]) for l in labels])) for t in range(T) for s in states] # Path2 state to labels
            +[Or([S1[0] == stateToNum[s] for s in states if 'Ss' in dts_labels[s]] )]
            + [Or([S2[0] == stateToNum[s] for s in states if 'Ss' in dts_labels[s]] )]
            +[l2[0][labelToNum['Ss']]]
            +[l1[0][labelToNum['Ss']]]
            + [S1[0] != S2[0]]
            + [l1[t][labelToNum[l]] == l2[t][labelToNum[l]] for t in range(T) for l in ['U', 'R', 'D', 'L']] # Action labels are same 
            + [Or([l1[t][labelToNum['G']] for t in range(T)])] # Finally Goal state in path2
            + [Or([l2[t][labelToNum['G']] for t in range(T)])]
            + [And([l1[t][labelToNum[l]] == l2[t][labelToNum[l]] for t in range(T) for l in ['U', 'R', 'D', 'L']])]
            + [ Exists(
                [l1[t_][labelToNum['G']] for t_ in range(T)],
                ForAll(
                    [S2[t] for t in range(T)],
                   Implies(
                       And([l1[t][labelToNum[l]] == l2[t][labelToNum[l]] for t in range(T) for l in ['U', 'R', 'D', 'L']]),
                        #Or([And(l1[t][labelToNum['G']],l2[t][labelToNum['G']]) for t in range(T)])
                        Or([l2[t][labelToNum['G']] for t in range(T)])

                    )
                    )
                )]

            )
        )
start = time.time()
res = solver.check()
end = time.time()
print(" Init state robustness solved in " + str(end - start) + " sec")
strategy = []
strategy1 = []
if res == sat:
    m = solver.model()
    print("Strategy: ")
    for t in range(T):
        s = numToState[m.evaluate(S2[t], model_completion=True).as_long()]
        s1 = numToState[m.evaluate(S1[t], model_completion=True).as_long()]
        _,i,j,l = s.split('_')
        _,i1,j1,l1= s1.split('_')
       # if 'G' in dts_labels[s1] and 'G' in dts_labels[s]:
       #     break

        strategy.append([l, i, j])
        strategy1.append([l1, i1, j1])

        print(l+" from state "+'s_'+i+'_'+j)


else:
    print("Does not satisfy")
    print(solver.unsat_core())
cdts.plot([], [], 'init_state_robustness_problem')
cdts.plot(strategy, [],'init_state_robustness_solution')
solver.pop()



