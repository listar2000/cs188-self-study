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

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    # marked is a dict that stores already visted states to avoid re-computation
    stack = util.Stack()
    # a node is a triple (location, direction, cost)
    initialNode = ([problem.getStartState()], [], 0)
    stack.push(initialNode)

    while not stack.isEmpty():
        currNode = stack.pop()
        currLocations = currNode[0]
        currLocation = currLocations[-1]
        if problem.isGoalState(currLocation):
            return currNode[1]
        else:
            for successor in problem.getSuccessors(currLocation):
                nextLocation, nextDir, nextCost = successor[0], successor[1], successor[2]
                if not nextLocation in currLocations:
                    newLoc = currNode[0] + [nextLocation]
                    newDir = currNode[1] + [nextDir]
                    newCost = currNode[2] + nextCost
                    newNode = (newLoc, newDir, newCost)
                    stack.push(newNode)
    return None
    
def getWholePath(actionTo, goal, problem):
    paths = []
    startState = problem.getStartState()

    while goal != startState:
        info = actionTo[goal]
        paths.append(info[1])
        goal = info[0]

    paths.reverse()
    return paths

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    # marked is a dict that stores already visted states to avoid re-computation
    queue = util.Queue()
    marked = set()
    startLoc = problem.getStartState()
    # a node is a triple (location, direction, cost)
    initialNode = (startLoc, [], 0)
    queue.push(initialNode)
    marked.add(startLoc)

    while not queue.isEmpty():
        currNode = queue.pop()
        currLocation = currNode[0]
        if problem.isGoalState(currLocation):
            return currNode[1]
        else:
            for successor in problem.getSuccessors(currLocation):
                nextLocation, nextDir, nextCost = successor[0], successor[1], successor[2]
                if not nextLocation in marked:
                    marked.add(nextLocation)
                    newDir = currNode[1] + [nextDir]
                    newCost = currNode[2] + nextCost
                    newNode = (nextLocation, newDir, newCost)
                    queue.push(newNode)
    return None

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    pq = util.PriorityQueue()
    actionTo = dict()
    start = problem.getStartState()
    pq.push(start, 0)
    actionTo[start] = (None, None, 0)

    while not pq.isEmpty():
        state = pq.pop()
        if problem.isGoalState(state):
            return getWholePath(actionTo, state, problem)
        else:
            for successor in problem.getSuccessors(state):
                cost = actionTo[state][2] + successor[2]
                target = successor[0]
                if not target in actionTo:
                    # actionTo[state][2] is the cost of the previous state
                    # successor[2] is the edge cost                    
                    pq.push(target, cost)
                    actionTo[target] = (state, successor[1], cost)
                else:
                    if cost < actionTo[target][2]:
                        actionTo[target] = (state, successor[1], cost)
    return None                   


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    pq = util.PriorityQueue()
    actionTo = dict()
    start = problem.getStartState()
    pq.push(start, 0)
    actionTo[start] = (None, None, 0)

    while not pq.isEmpty():
        state = pq.pop()
        if problem.isGoalState(state):
            return getWholePath(actionTo, state, problem)
        else:
            for successor in problem.getSuccessors(state):
                cost = actionTo[state][2] + successor[2]
                target = successor[0]
                if not target in actionTo:
                    # actionTo[state][2] is the cost of the previous state
                    # successor[2] is the edge cost                    
                    pq.push(target, cost + heuristic(target, problem))
                    actionTo[target] = (state, successor[1], cost)
                else:
                    if cost < actionTo[target][2]:
                        actionTo[target] = (state, successor[1], cost)
                        pq.update(target, cost + heuristic(target, problem))
    return None        


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
