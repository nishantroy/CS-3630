# author1: Nishant Roy
# author2: Madelyn Juby

from visualizer import *
import numpy
from queue import PriorityQueue
import cozmo
from cozmo.util import degrees
from grid import *


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

    robot.set_lift_height(1000).wait_for_completed()
    # robot.set_lift_height(-1000).wait_for_completed()
    robot.set_head_angle(degrees(0)).wait_for_completed()

    goalFound = False

    cozmoX = 3
    cozmoY = 2
    cozmoDirection = 0

    for i in range(0, 26):
        grid.addObstacle((i, 0))
        grid.addObstacle((i, 17))

    for j in range(0, 18):
        grid.addObstacle((0, j))
        grid.addObstacle((25, j))

    cube2NotFound = True
    cube3NotFound = True

    if robot.world.light_cubes[cozmo.objects.LightCube1Id].is_visible:
        print("found it!")
        cube1ObservedX = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.position.x
        cube1ObservedY = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.position.y

        zAngle = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.rotation.angle_z.radians
        print(str(zAngle))

        goalX = numpy.round((cube1ObservedX + (math.cos(zAngle) * 25)) / 25)
        goalY = numpy.round((cube1ObservedY + (math.sin(zAngle) * 25)) / 25)
        goal = (goalX, goalY)
        cube1X = cube1ObservedX / 25
        cube1X = numpy.round(cube1X)
        cube1Y = cube1ObservedX / 25
        cube1Y = numpy.round(cube1Y)
        cube1X += 3
        cube1Y += 2

        for i in range(-1, 2):
            for j in range(-1, 2):
                grid.addObstacle((cube1X + i, cube1Y + j))

        # grid.addObstacles(obstacles)
        # grid.setGoal((cube1X, cube1Y))
        # goalX = cube1X
        # goalY = cube1Y
        # if zAngle == -2:
        #     # Go to left of seen face
        #     pass
        # elif zAngle == 2:
        #     # Go to right of seen face
        #     pass
        # elif zAngle == 3 or zAngle == -3:
        #     # Go to opposite side
        #     pass
        # else:
        #     # Go to this face
        #     pass
        # if cube1X < cozmoX:
        print("Goal: " + str(goal))
        grid.clearGoals()
        grid.addGoal(goal)
        # else:
        #     grid.addGoal((cube1X, cube1Y))
        grid.clearStart()
        grid.setStart((cozmoX, cozmoY))
        grid.clearVisited()
        astar(grid, heuristic)
        path = grid.getPath()
        pathIndex = 0
        goalFound = True
    else:
        grid.clearGoals()
        grid.addGoal((12, 8))
        grid.clearVisited()
        astar(grid, heuristic)
        path = grid.getPath()
        pathIndex = 0

    actualCube = (0, 0)

    while not stopevent.is_set():
    #     if robot.world.light_cubes[cozmo.objects.LightCube1Id].is_visible:
    #         zAngle = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.rotation.angle_z.degrees
    #         if zAngle > 0:
    #             zAngle -= 180
    #         else:
    #             zAngle += 180
    #
    #         # print(str(zAngle))
    #     robotsPose = (robot.pose.position.x, robot.pose.position.y)
    #     robotsAngle = robot.pose_angle.degrees
    #     print(str(robotsPose))
    #     print(str(robotsAngle))
    #     robot.drive_straight(cozmo.util.distance_mm(25), cozmo.util.speed_mmps(30)).wait_for_completed()
    #     robot.turn_in_place(cozmo.util.degrees(270)).wait_for_completed()
    #     robotsPose = (robot.pose.position.x, robot.pose.position.y)
    #     robotsAngle = robot.pose_angle.degrees
    #     print(str(robotsPose))
    #     print(str(robotsAngle))
    #     break

        if cube2NotFound and robot.world.light_cubes[cozmo.objects.LightCube2Id].is_visible:
            print("Found Cube 2")
            cube2NotFound = False

            cube2ObservedX = robot.world.light_cubes[cozmo.objects.LightCube2Id].pose.position.x
            cube2ObservedY = robot.world.light_cubes[cozmo.objects.LightCube2Id].pose.position.y

            cube2X = cube2ObservedX
            cube2Y = cube2ObservedY
            cube2X /= 25
            cube2X = numpy.round(cube2X)
            cube2Y /= 25
            cube2Y = numpy.round(cube2Y)
            cube2X += 3
            cube2Y += 2

            for i in range(-2, 3):
                for j in range(-2, 3):
                    grid.addObstacle((cube2X + i, cube2Y + j))

            grid.clearStart()
            grid.setStart((cozmoX, cozmoY))
            grid.clearVisited()
            astar(grid, heuristic)
            path = grid.getPath()
            pathIndex = 0

        if cube3NotFound and robot.world.light_cubes[cozmo.objects.LightCube3Id].is_visible:
            print("Found Cube 3")
            cube3NotFound = False
            cube3ObservedX = robot.world.light_cubes[cozmo.objects.LightCube3Id].pose.position.x
            cube3ObservedY = robot.world.light_cubes[cozmo.objects.LightCube3Id].pose.position.y

            cube3X = cube3ObservedX
            cube3Y = cube3ObservedY
            cube3X /= 25
            cube3X = numpy.round(cube3X)
            cube3Y /= 25
            cube3Y = numpy.round(cube3Y)
            cube3X += 3
            cube3Y += 2
            for i in range(-2, 3):
                for j in range(-2, 3):
                    grid.addObstacle((cube3X + i, cube3Y + j))

            grid.clearStart()
            grid.setStart((cozmoX, cozmoY))
            grid.clearVisited()
            astar(grid, heuristic)
            path = grid.getPath()
            pathIndex = 0

        if not goalFound and robot.world.light_cubes[cozmo.objects.LightCube1Id].is_visible:
            print("No goal found, but seen it now")
            cube1ObservedX = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.position.x
            cube1ObservedY = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.position.y

            print(str((cube1ObservedX, cube1ObservedY)))

            zAngle = robot.world.light_cubes[cozmo.objects.LightCube1Id].pose.rotation.angle_z.radians
            print("Before correction: " + str(zAngle))

            if zAngle > 0:
                zAngle -= math.pi
            elif zAngle < 0:
                zAngle += math.pi

            print("After correction: " + str(zAngle))

            # goalX = numpy.round((cube1ObservedX + (math.cos(zAngle) * 50)) / 25) + 3
            # goalY = numpy.round((cube1ObservedY + (math.sin(zAngle) * 50)) / 25) + 2
            # goal = (goalX, goalY)
            cube1X = cube1ObservedX / 25
            cube1X = numpy.round(cube1X)
            cube1Y = cube1ObservedY / 25
            cube1Y = numpy.round(cube1Y)
            cube1X += 3
            cube1Y += 2

            actualCube = (cube1X, cube1Y)

            goalX = numpy.round(cube1X + (numpy.round((math.cos(zAngle))) * 4))
            goalY = numpy.round(cube1Y + (numpy.round((math.sin(zAngle))) * 4))
            if goalX <= 0:
                goalX = 1
            elif goalX >= 25:
                goalX = 24

            if goalY <= 0:
                goalY = 1
            elif goalY >= 17:
                goalY = 16

            goal = (goalX, goalY)

            print("Saw it at : " + str((cube1X, cube1Y)))

            print("Going to: " + str(goal))
            grid.clearVisited()
            grid.clearGoals()
            for i in range(-2, 3):
                for j in range(-2, 3):
                    obstacle = (cube1X + i, cube1Y + j)
                    if grid.coordInBounds(obstacle) is True and obstacle != goal:
                        grid.addObstacle(obstacle)

            grid.addGoal(goal)

            grid.clearStart()
            grid.setStart((cozmoX, cozmoY))
            astar(grid, heuristic)
            path = grid.getPath()
            pathIndex = 0
            goalFound = True


        else:
            if pathIndex < len(path):
                nextSquare = path[pathIndex]
                print("Next: " + str(nextSquare))
                print("Now: " + str((cozmoX, cozmoY)))
                pathIndex += 1
                cozmoDirection = moveToBox((cozmoX, cozmoY), nextSquare, cozmoDirection, robot)
                cozmoX = nextSquare[0]
                cozmoY = nextSquare[1]
            else:
                if not goalFound:
                    robot.turn_in_place(degrees(45)).wait_for_completed()
                    cozmoDirection -= 45
                    if cozmoDirection < 0:
                        cozmoDirection += 360
                    print("Searching from center!")
                else:
                    print("At goal!")
                    nextDirection = 0
                    if cozmoX == actualCube[0]:
                        if cozmoY < actualCube[1]:
                            nextDirection = 270
                            # Go 25 in +y
                            pass
                        elif cozmoY > actualCube[1]:
                            nextDirection = 90
                            # Go 25 in -y
                            pass
                        else:
                            nextDirection = cozmoDirection
                            return cozmoDirection
                    elif cozmoY == actualCube[1]:
                        if cozmoX < actualCube[0]:
                            # Go 25 in +x
                            nextDirection = 0
                            pass
                        elif cozmoX > actualCube[0]:
                            nextDirection = 180
                            # Go 25 in -x
                            pass
                    elif cozmoY < actualCube[1]:
                        if cozmoX < actualCube[0]:
                            # Go northwest
                            nextDirection = 315
                            pass
                        elif cozmoX > actualCube[0]:
                            nextDirection = 225
                            # Go southwest
                            pass
                    elif cozmoX < actualCube[0]:
                        if cozmoY > actualCube[1]:
                            # Go northeast
                            nextDirection = 45
                            pass
                    elif cozmoX > actualCube[0] and cozmoY > actualCube[1]:
                        # Go southeast
                        nextDirection = 135
                        pass
                    turnAngle = turnToFace(cozmoDirection, nextDirection)
                    robot.turn_in_place(degrees(turnAngle)).wait_for_completed()
                    stopevent.set()
                    # reached


#
# def rotateCoordinates(x, y, cwAngle):
#     x //= 25
#     y //= 25
#
#     newCoords = (x, y)
#     return newCoords


def getAngleToTurn(currentAngle, current, next):
    nextY = next[1] * 25
    currentY = current[1] * 25
    nextX = next[0] * 25
    currentX = current[0] * 25
    angle = math.atan2((nextY - currentY + 20), (nextX - currentX + 20))
    angle = math.degrees(angle)

    pass


def turnToFace(current, nextDirection):
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


def moveToBox(cozmoCoord, boxCoord, cozmoDirection, robot):
    nextDirection = 0
    boxX = boxCoord[0]
    boxY = boxCoord[1]
    cozmoX = cozmoCoord[0]
    cozmoY = cozmoCoord[1]

    if cozmoX == boxX:
        if cozmoY < boxY:
            nextDirection = 270
            # Go 25 in +y
            pass
        elif cozmoY > boxY:
            nextDirection = 90
            # Go 25 in -y
            pass
        else:
            nextDirection = cozmoDirection
            return cozmoDirection
    elif cozmoY == boxY:
        if cozmoX < boxX:
            # Go 25 in +x
            nextDirection = 0
            pass
        elif cozmoX > boxX:
            nextDirection = 180
            # Go 25 in -x
            pass
    elif cozmoY < boxY:
        if cozmoX < boxX:
            # Go northwest
            nextDirection = 315
            pass
        elif cozmoX > boxX:
            nextDirection = 225
            # Go southwest
            pass
    elif cozmoX < boxX:
        if cozmoY > boxY:
            # Go northeast
            nextDirection = 45
            pass
    elif cozmoX > boxX and cozmoY > boxY:
        # Go southeast
        nextDirection = 135
        pass
    turnAngle = turnToFace(cozmoDirection, nextDirection)
    robot.turn_in_place(degrees(turnAngle)).wait_for_completed()
    cozmoDirection = nextDirection
    if cozmoDirection == 0 or cozmoDirection == 90 or cozmoDirection == 180 or cozmoDirection == 270:
        robot.drive_straight(cozmo.util.distance_mm(25), cozmo.util.speed_mmps(25)).wait_for_completed()
    else:
        robot.drive_straight(cozmo.util.distance_mm(25 * math.sqrt(2)), cozmo.util.speed_mmps(25)).wait_for_completed()
    return cozmoDirection


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
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
