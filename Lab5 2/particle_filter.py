from grid import *
from particle import Particle
from utils import *
from setting import *
from sys import *


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief ~{p}(x_{t} | u_{t})
                after motion update
    """
    if all(odom) == 0:
        return particles

    # print(particles)
    # print(odom)
    for particle in particles:
        (x, y) = rotate_point(odom[0], odom[1], particle.h)
        particle.x += x
        particle.y += y
        particle.h += odom[2]
        particle.h %= 360
        particle.x += add_gaussian_noise(particle.x, ODOM_TRANS_SIGMA)
        particle.y += add_gaussian_noise(particle.y, ODOM_TRANS_SIGMA)
        particle.h += add_gaussian_noise(particle.h, ODOM_HEAD_SIGMA)

    return particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    pairings = list()

    # Step 1: Set weights ( I THINK!)
    # TODO: Check Step 1, and do Step 2. Can't test anything till this is done.
    for cm in measured_marker_list:
        cmX = cm[0]
        cmY = cm[1]
        shortest = [cm, None]
        minDist = maxsize
        for particle in particles:
            markers_visible_to_particle = particle.read_markers(grid)
            index = -1
            toRem = index
            for visible in markers_visible_to_particle:
                index += 1
                visX = visible[0]
                visY = visible[1]
                dist = grid_distance(cmX, cmY, visX, visY)
                if shortest[1] is None or dist < minDist:
                    minDist = dist
                    shortest[1] = visible
                    toRem = index
            del(markers_visible_to_particle[toRem])
            pairings.append(shortest)



    measured_particles = []
    return measured_particles


