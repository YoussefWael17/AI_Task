from util import *
import time, os
import traceback
import sys
import sys
import math
import random
import string
import time
import types
import tkinter
import os.path
from game import GameStateData
from game import Game
from game import Directions
from game import Actions
from util import nearestPoint
from util import manhattanDistance
import util, layout
import sys, types, time, random, os


# question 1
#python pacman.py -l mediumMaze -p SearchAgent -a fn=breadthFirstSearch
#python pacman.py -l bigMaze -p SearchAgent -a fn=breadthFirstSearch -z .5
def breadthFirstSearch(problem):
    queue = util.Queue()  # Create an empty queue
    visited = set()  # Create an empty set for visited states

    # Enqueue the initial state
    queue.push((problem.getStartState(), []))

    while not queue.isEmpty():
        state, path = queue.pop()

        # If the popped state is the goal state, return the path
        if problem.isGoalState(state):
            return path

        # Add the popped state to the visited set
        visited.add(state)

        # Expand the current state
        successors = problem.getSuccessors(state)

        for successor in successors:
            successorState = successor[0]
            action = successor[1]

            # If the successor state has not been visited, enqueue it
            if successorState not in visited:
                queue.push((successorState, path + [action]))
                visited.add(successorState)  # Mark the state as visited to avoid duplicates

    return []  # Return an empty path if no goal state is found


#question 2
#python pacman.py -l mediumMaze -p SearchAgent -a fn=breadthFirstSearch
#python pacman.py -l bigMaze -p SearchAgent -a fn=breadthFirstSearch -z .5
def breadthFirstSearch(problem):
    queue = util.Queue()  # Create an empty queue
    visited = set()  # Create an empty set for visited states

    # Enqueue the initial state
    queue.push((problem.getStartState(), []))

    while not queue.isEmpty():
        state, path = queue.pop()

        # If the popped state is the goal state, return the path
        if problem.isGoalState(state):
            return path

        # Add the popped state to the visited set
        visited.add(state)

        # Expand the current state
        successors = problem.getSuccessors(state)

        for successor in successors:
            successorState = successor[0]
            action = successor[1]

            # If the successor state has not been visited, enqueue it
            if successorState not in visited:
                queue.push((successorState, path + [action]))
                visited.add(successorState)  # Mark the state as visited to avoid duplicates

    return []  # Return an empty path if no goal state is found



# question 3
#python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=aStarSearch,heuristic=manhattanHeuristic
def aStarSearch(problem, heuristic):
    priorityQueue = util.PriorityQueue()  # Create an empty priority queue
    visited = set()  # Create an empty set for visited states

    # Enqueue the initial state with priority based on the heuristic value
    priorityQueue.push((problem.getStartState(), []), heuristic(problem.getStartState(), problem))

    while not priorityQueue.isEmpty():
        state, path = priorityQueue.pop()

        # If the popped state is the goal state, return the path
        if problem.isGoalState(state):
            return path

        # Add the popped state to the visited set
        visited.add(state)

        # Expand the current state
        successors = problem.getSuccessors(state)

        for successor in successors:
            successorState = successor[0]
            action = successor[1]
            cost = successor[2]

            # If the successor state has not been visited, enqueue it with priority based on the heuristic value and path cost
            if successorState not in visited:
                newPath = path + [action]
                priority = problem.getCostOfActions(newPath) + heuristic(successorState, problem)
                priorityQueue.push((successorState, newPath), priority)
                visited.add(successorState)  # Mark the state as visited to avoid duplicates

    return []  # Return an empty path if no goal state is found


# question 4
#python pacman.py -l tinyCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
#python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem

class CornersProblem:
    def __init__(self, startingGameState):
        self.startingPosition = startingGameState.getPacmanPosition()
        self.corners = self.getCorners(startingGameState)
        self.goal = tuple([False] * 4)  # Goal state representing touching all four corners

    def getCorners(self, gameState):
        walls = gameState.getWalls()
        corners = [(1, 1), (1, walls.height - 2), (walls.width - 2, 1), (walls.width - 2, walls.height - 2)]
        return corners

    def getStartState(self):
        return (self.startingPosition, tuple([False] * 4))

    def isGoalState(self, state):
        return state[1] == self.goal

    def getSuccessors(self, state):
        successors = []
        position, visitedCorners = state
        x, y = position

        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            dx, dy = Actions.directionToVector(action)
            nextX, nextY = int(x + dx), int(y + dy)
            hitsWall = self.hasWall(nextX, nextY)

            if not hitsWall:
                nextPosition = (nextX, nextY)
                newVisitedCorners = list(visitedCorners)

                for i, corner in enumerate(self.corners):
                    if nextPosition == corner:
                        newVisitedCorners[i] = True

                successors.append(((nextPosition, tuple(newVisitedCorners)), action, 1))

        return successors

    def hasWall(self, x, y):
        walls = self.walls
        return walls[x][y]


