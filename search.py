# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import searchAgents

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    A sample depth first search implementation is provided for you to help you understand how to interact with the problem.
    """
    
    mystack = util.Stack()
    startState = (problem.getStartState(), '', 0, [])
    mystack.push(startState)
    visited = set()
    #state = mystack.pop()
    #while mystack : #while not mystack.isEmpty():
    while not mystack.isEmpty():

        state = mystack.pop() #拿掉變空啦(只有第一次會變空)#走到底要倒退，倒退的方法是移除最後一個node (pop) #拿掉新加的state

        node, action, cost, path = state #拿出來時node已經變成下一個
        ##--------------------------
        #if node in visited :
        #    state = mystack.pop()
        ##-------------------------
        if node not in visited : #走過的就不會再走一次
            visited.add(node)    #沒走過，此node加入visited
            if problem.isGoalState(node) :
                path = path + [(node, action)]
                break;
            succStates = problem.getSuccessors(node) #拿接續的state (可能多個)
            #print(succStates)
            for succState in succStates :
                succNode, succAction, succCost = succState
                newstate = (succNode, succAction, cost + succCost, path + [(node, action)])
                mystack.push(newstate) #mystack加了接續的多個state
        ###for check
        #aaa = mystack.pop()
        #print(aaa)###
        #mystack.push(aaa)
        ###
    actions = [action[1] for action in path] #initialize the actions list / 其實for "action"的"action"是拿出(node,action)
    '''
    actions=[]
    for action in path:
        actions.append(action[1])
    '''

    del actions[0] #delete, because the first aciton is '' 第一個node沒有action
    return actions

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    myQueue = util.Queue()
    startState = (problem.getStartState(), '', 0, [])
    myQueue.push(startState)
    visited = set()
    while not myQueue.isEmpty():
        expandState = myQueue.pop()
        node, action, cost, path = expandState
        #print(cost)
        if node not in visited:
            visited.add(node)
            if problem.isGoalState(node):
                path = path + [(node, action)]
                break;
            succStates = problem.getSuccessors(node)
            for succState in succStates:
                succNode, succAction, succCost = succState
                newstate = (succNode, succAction, cost + succCost, path + [(node, action)])
                myQueue.push(newstate)
    actions = [action[1] for action in path]
    #print(node)
    del actions[0]
    return actions

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def enforcedHillClimbing(problem, heuristic=nullHeuristic): #heuristic=util.manhattanDistance
    """COMP90054 your solution to part 1 here """
    startState = (problem.getStartState(), '', 0, [])
    tempActions = []
    actions = []
    #print(startState[0])
    
    #node1, action1, cost1, path1 = startState
    while not problem.isGoalState(startState[0]):
        startState = improve(startState,problem)
    if problem.isGoalState(startState[0]):    
        node2, action2, cost2, path2 = startState
        path2 = path2 + [(node2, action2)]
        for action in path2:
            actions.append(action[1])
    
    #while '' in actions:
    #    actions.remove('')
    #actions.reverse()
    #print(actions)
    print(path2)
    del actions[0]
    return actions

def improve(state0,problem):
    node0, action0, cost0, path0 = state0
    myQueue = util.Queue()
    myQueue.push(state0)
    visited = set()
    while not myQueue.isEmpty():
        popState = myQueue.pop()
        node, action, cost, path = popState
        if node not in visited:
            visited.add(node)
            if searchAgents.manhattanHeuristic(node,problem) < searchAgents.manhattanHeuristic(node0,problem):
                #break;
                return popState #current state
            succStates = problem.getSuccessors(node)
            for succState in succStates:
                succNode, succAction, succCost = succState
                newstate = (succNode, succAction, cost + succCost, path + [(node, action)])
                myQueue.push(newstate)
    #actions = [action[1] for action in path]
    #del actions[0]
    return popState


'''
def initialNodeBFS(initialstate):
    myQueue = util.Queue()
    expandedQueue = util.Queue()
    myQueue.push(initialstate)
    visited = set()
    while not myQueue.isEmpty():
        expandState = myQueue.pop()
        ##expendedQueue.push(expandState)
        node, action, cost, path = expandState
        #print(node)
        if node not in visited:
            visited.add(node)
            if problem.isGoalState(node):
                path = path + [(node, action)]
                break;
            succStates = problem.getSuccessors(node)
            for succState in succStates:
                succNode, succAction, succCost = succState
                newstate = (succNode, succAction, cost + succCost, path + [(node, action)])
                myQueue.push(newstate)
    actions = [action[1] for action in path]
    del actions[0]
    return expandedQueue
    '''
   
def idaStarSearch(problem, heuristic=nullHeuristic):
    """COMP90054 your solution to part 2 here """
    startState = (problem.getStartState(), '', 0, [])
    threshold = searchAgents.manhattanHeuristic(startState[0],problem)
    route = util.Stack()
    route.push(startState)

    while True:
        temp = searchF(route,0,threshold,problem) #at first state, the g is 0
        if temp == FOUND:
            finalState = route.pop()
            fnode, faction, fcost, fpath = finalState
            fpath = fpath + [(fnode,faction)] ###???????????????????????????? path到底要加上啥阿
            actions = [faction[1] for faction in fpath]
            del actions[0]
            return actions
            #return print('yes')
            #return path
        if temp == float("inf"):
            return Fail
        threshold = temp


def searchF(route, g, threshold, problem):
    currState = route.pop()
    route.push(currState)##放回去?
    #print(currState)###################
    node, action, cost, path = currState
    f = g + searchAgents.manhattanHeuristic(node,problem)
    if f > threshold:
        # 在idaStarSearch中，會把這個較大的f當作新的threshold
        return f
    if problem.isGoalState(node):
        #path = path + [(node,action)]
        return FOUND
    succStates = problem.getSuccessors(node)
    min = float("inf")
    for succState in succStates: #find the smallerst f cost of children
        #recursive call with next node as current node for depth search
        #if succState not in route: #route是stack，不能這樣用
        if True:
            succNode, succAction, succCost = succState
            newstate = (succNode, succAction, cost + succCost, path + [(node, action)])
            route.push(newstate)
            temp = searchF(route, g+1, threshold, problem) #cost between node is 1
            if (temp == FOUND):
                return FOUND
            if (temp < min):
                min = temp
            route.pop()

    return min


                
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ehc = enforcedHillClimbing
ida = idaStarSearch
FOUND = 94879487