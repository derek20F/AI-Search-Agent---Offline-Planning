def idaStarSearch(problem, heuristic=nullHeuristic):
    """COMP90054 your solution to part 2 here """
    startState = (problem.getStartState(), '', 0, [])
    thershold = searchAgemts.manhattanHeuristic(startState[0],problem)
    route = util.Stack()
    route.push(startState)

    while True:
    	temp = searchF(route,0,threshold,problem) #at first state, the g is 0
    	if temp == FOUND
    		return print('yes')
    		#return path
    	if temp == float("inf")
    		return Fail
    	threshold = temp


def searchF(route, g, threshold, problem):
	currState = route.pop()
	route.push(currState)##放回去?
	node, action, cost path = currState
	f = g + searchAgents.manhattanHeuristic(node,problem)
	if f > threshold:
		# 在idaStarSearch中，會把這個較大的f當作新的threshold
		return f
	if problem.isGoalState(node):
		return FOUND
	succStates = problem.getSuccessors(node)
    min = float("inf")
    for succState in succStates: #find the smallerst f cost of children
		#recursive call with next node as current node for depth search
		if succState not if route:
			route.push(succState)
			temp = searhF(route, g+1, threshold) #cost between node is 1
			if (temp == FOUND):
				return FOUND
			if (temp < min):
				min = temp
			route.pop()

	return min
			#succNode, succAction, succCost = succState
			#newstate = (succNode, succAction, cost + succCost, path + [(node, action)])
			
	#newstate = (succNode, succAction, cost + succCost, path + [(node, action)])
	#這樣是不是只有找到頭和尾吧，之間的path有建立嗎?



    #myPQ = util.PriorityQueue()
    
    #myPQ.push(startState)
    #visited = set()
    #while not myPQ.isEmpty():
        


    #del actions[0]
    #return actions