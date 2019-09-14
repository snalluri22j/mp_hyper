import numpy as np
import matplotlib.pyplot as plt
class CDTS:
    def __init__(self, S: list, A: list, T, L: dict):
        self.S = S
        self.A = A
        self.T = T
        self.L = L
    def getStates(self):
        return self.S
    def getActions(self):
        return self.A
    def getTransFunction(self,s,a):
        return self.T(s,a)
    def getLabels(self):
        return self.L
    def print(self):
        print("~~~~~DTS~~~~")
        print("States:")
        for s in self.S:
            print(s,end=' ')
        print()
        print("Actions:")
        for a in self.A:
            print(a,end=' ')
        print()
        print("Transitions:")
        for s in self.S:
            for a in self.A:
                if self.T(s,a):
                    print(s+' x '+a+' -> '+self.T(s,a))
        print("Labels:")
        for s in self.S:
            lab = ''
            for x in self.L[s]:
                lab = lab + x + ' '
            print(s+" : "+lab)

    def plot(self, strategy1, strategy2, fname):
        N = int(np.sqrt(len(self.S)))
        gridData = np.ones((N,N,3))
        for s in self.S:
            _,i,j = s.split("_")
            for l in self.L[s]:
                if l.startswith('O'):
                    gridData[int(i)][int(j)][:] = (1-0.05*int(l[1:]))*np.array([1.0,1.0,1.0])

            if 'X' in self.L[s]:
                gridData[int(i)][int(j)][:] = [0, 0, 0]
            if 'Ss' in self.L[s]:
                gridData[int(i)][int(j)][:] = [1, 0, 0]
            if 'S' in self.L[s]:
                gridData[int(i)][int(j)][:] = [1, 0, 0]

            if 'G' in self.L[s]:
                gridData[int(i)][int(j)][:] = [0, 1, 0]

        fig = plt.figure()
        gridPlot = plt.imshow(gridData)
        ax = gridPlot._axes
        ax.grid(visible=True)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.set_xlim([-0.5,N-0.5])
        ax.set_ylim([-0.5,N-0.5])
        ax.set_xticks(np.arange(-0.5, N, 1))
        ax.set_yticks(np.arange(-0.5, N, 1))
        for x in strategy1:
            [l,i,j] = x
            dx = 0
            dy = 0
            if l == 'D':
                dx = 0
                dy = -1
            if l == 'U':
                dx = 0
                dy = 1
            if l == 'R':
                dx =  1
                dy = 0
            if l == 'L':
                dx = -1
                dy = 0
            ax.arrow(int(j), int(i), dx, dy, color='b', shape='full', head_width=0.2, length_includes_head=True, alpha=0.5)

        for x in strategy2:
            [l,i,j] = x
            dx = 0
            dy = 0
            if l == 'D':
                dx = 0
                dy = -1
            if l == 'U':
                dx = 0
                dy = 1
            if l == 'R':
                dx =  1
                dy = 0
            if l == 'L':
                dx = -1
                dy = 0
            ax.arrow(int(j), int(i), dx, dy, color='m', shape='full', head_width=0.2, length_includes_head=True, alpha=0.5)


        plt.savefig("../plots/"+fname+".png", bbox_inches='tight', pad_inches=0)


class DTS:
    S = []
    T = {}
    L = {}
    
    def __init__(self, M: CDTS):
        for s in M.getStates():
            for a in M.getActions():
                s_ = M.getTransFunction(s,a)
                if s_ != []:
                    self.S.append(s+"_"+a)
        # Add labels
        for s in self.S:
            self.L[s] = []
        labels = M.getLabels()
        for k in labels.keys():
            for s in self.S:
                if s.startswith(k):
                    for x in labels[k]:
                        self.L[s].append(x)
                    #self.L[s].append(k)

        for s in M.getStates():
            for a in M.getActions():
                if M.getTransFunction(s,a):
                    self.T[s+"_"+a] = []
                s_ = M.getTransFunction(s,a)
                if s_ != []:
                    for a_ in M.getActions():
                        if s_+"_"+a_ in self.S:
                            #if 'X' in self.L[s_+"_"+a_]:
                            #    self.L[s+"_"+a].append('NX')
                            self.T[s+"_"+a].append(s_+"_"+a_)
                            if a not in self.L[s+"_"+a]:
                                self.L[s+"_"+a].append(a)
        for s in M.getStates():
            for a in M.getActions():
                if M.getTransFunction(s,a) and 'G' in self.L[s+"_"+a]:
                    self.T[s+"_"+a].append(s+"_"+a)
        #print(self.L)

    def getStates(self):
        return self.S
    def getTrans(self):
        return self.T
    def getLabels(self):
        return self.L
    def print(self):
        print("~~~~~~DTS~~~~~")
        print("States:")
        for s in self.S:
            print(s,end=' ')
        print()
        print("Transition:")
        for t in self.T:
            print(t, self.T[t])
        print("Labels:")
        for s in self.S:
            lab = ''
            for x in self.L[s]:
                lab = lab + x + ' '
            print(s+" : "+lab)


