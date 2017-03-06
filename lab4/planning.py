# author1: Nishant Roy
# author2: Madelyn Juby

from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from cozmo.util import degrees, distance_mm
from grid import *
from enum import Enum


def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """

    start = grid.getStart()
    curr = (0, start, 0)
    goals = grid.getGoals()[0]
    q = PriorityQueue()

    parents = {}

    while curr[1] != goals:
        grid.addVisited(curr[1])
        for neighbor in grid.getNeighbors(curr[1]):
            coord = neighbor[0]
            weight = neighbor[1]
            distance = curr[2] + weight
            if coord not in grid.getVisited():
                heurVal = heuristic(coord, goals)
                q.put((distance + heurVal, coord, distance))
                if coord not in parents.keys():
                    parents[coord] = (curr[1], distance)
                else:
                    if parents[coord][1] > distance:
                        parents[coord] = (curr[1], distance)

        curr = q.get()
        while curr[1] in grid.getVisited():
            curr = q.get()

    path = []
    curr = curr[1]
    while curr != start:
        path.append(curr)
        curr = parents[curr][0]

    path.append(start)
    path = path[::-1]

    grid.setPath(path)


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    x = int(current[0])
    y = int(current[1])
    a = int(goal[0])
    b = int(goal[1])
    euclDist = math.sqrt((x - a) ** 2 + (y - b) ** 2)
    manDist = abs(x - a) + abs(y - b)

    return euclDist


def test():
    global grid, stopevent
    cozmoX = 3
    cozmoY = 2

    for i in range(0, 26):
        grid.addObstacle((i, 0))
        grid.addObstacle((i, 17))

    for j in range(0, 18):
        grid.addObstacle((0, j))
        grid.addObstacle((25, j))

    grid.addGoal((13, 8))

    while not stopevent.is_set():
        grid.clearVisited()
        astar(grid, heuristic)
        # print(grid.checkPath())


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment document for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """

    global grid, stopevent

    robot.set_lift_height(0).wait_for_completed()
    robot.set_head_angle(degrees(-5)).wait_for_completed()

    goalFound = False
    noObstacle = True

    boxWidth = 25.4
    boxHeight = 25.4
    boxDiag = 25.4 * math.sqrt(2)

    cozmoX = 3
    cozmoY = 2
    cozmoDirection = 0

    for i in range(0, 26):
        grid.addObstacle((i, 0))
        grid.addObstacle((i, 17))

    for j in range(0, 18):
        grid.addObstacle((0, j))
        grid.addObstacle((25, j))

    path = []
    pathIndex = 0

    if robot.world.light_cubes[cozmo.objects.LightCube1Id].is_visible:
        goalFound = True
        cube1X = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.position.x
        cube1Y = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.position.y
        cube1X //= 25.4
        cube1Y //= 25.4
        cube1X += cozmoX
        cube1Y += cozmoY
        grid.addGoal((cube1X, cube1Y))
        astar(grid, heuristic)
        path = grid.getPath()
        pathIndex = 0

        cube2NotFound = True
        cube3NotFound = True


    while not stopevent.is_set():

        if robot.world.light_cubes[cozmo.objects.LightCube2Id].is_visible:
            cube2NotFound = False

            pass

        if robot.world.light_cubes[cozmo.objects.LightCube3Id].is_visible:
            cube3NotFound = False
            pass

        if not goalFound and robot.world.light_cubes[cozmo.objects.LightCube1Id].is_visible:
            cube1X = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.position.x
            cube1Y = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.position.y
            if cozmoDirection % 90 == 0:
                cube1X //= 25.4
                cube1Y //= 25.4
                cube1X += cozmoX
                cube1Y += cozmoY
                grid.addGoal((cube1X, cube1Y))
                grid.clearStart()
                grid.addStart((cozmoX, cozmoY))
                astar(grid, heuristic)
                path = grid.getPath()
                pathIndex = 0
            else:
                if cube1Y < 0:
                    robot.turn_in_place(degrees(45))
                    cozmoDirection -= 45
                    cozmoDirection %= 360
                else:
                    robot.turn_in_place(degrees(-45))
                    cozmoDirection += 45
                    cozmoDirection %= 360
                continue


def TurnToFace(current, nextDirection):
    turnValue = nextDirection - current

    if turnValue > 0:
        if turnValue > 180:
            turnValue = 360 - turnValue
        else:
            turnValue *= -1
    elif turnValue < 0:
        if turnValue < -180:
            turnValue = -360 - turnValue
        else:
            turnValue *= -1

    return turnValue


def moveToBox(cozmoCoord, boxCoord, cozmoDirection):
    leftmove = 0
    rightmove = 0
    boxX = boxCoord[0]
    boxY = boxCoord[1]
    cozmoX = cozmoCoord[0]
    cozmoY = cozmoCoord[1]

    if cozmoX == boxX:
        if cozmoY < boxY:
            # Go 25 in +y
            pass
        elif cozmoY > boxY:
            # Go 25 in -y
            pass
    elif cozmoY == boxY:
        if cozmoX < boxX:
            # Go 25 in +x
            pass
        elif cozmoX > boxX:
            # Go 25 in -x
            pass
    elif cozmoY < boxY:
        if cozmoX < boxX:
            # Go northwest
            pass
        elif cozmoX > boxX:
            # Go southwest
            pass
    elif cozmoX < boxX:
        if cozmoY > boxY:
            # Go northeast
            pass
    elif cozmoX > boxX and cozmoY > boxY:
        # Go southeast
        pass


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # test()
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()
