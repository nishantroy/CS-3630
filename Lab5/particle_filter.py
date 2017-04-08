#Madelyn Juby, Nishant Roy

from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []

    dx, dy, dh = odom

    for particle in particles:
        part_x, part_y, part_h = particle.xyh
        h = add_gaussian_noise(part_h + dh, ODOM_HEAD_SIGMA)

        dx_new, dy_new = rotate_point(add_gaussian_noise(dx, ODOM_TRANS_SIGMA), add_gaussian_noise(dy, ODOM_TRANS_SIGMA), h)

        motion_particles.append(Particle(part_x + dx_new, part_y + dy_new, h))

    return motion_particles

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
    def get_closest_part_marker(robot_marker, markers_seen_by_particle):
        rm_x, rm_y = robot_marker[0], robot_marker[1]
        closest = markers_seen_by_particle[0]
        min_distance = grid_distance(rm_x, rm_y, closest[0], closest[1])
        for marker in markers_seen_by_particle:
            pm_x, pm_y = marker[0], marker[1]
            dist = grid_distance(rm_x, rm_y, pm_x, pm_y)
            if dist < min_distance:
                closest = marker
                min_distance = dist

        return closest

    particle_weights = []
    for particle in particles:
        part_x, part_y = particle.xy

        if grid.is_free(part_x, part_y):

            if len(measured_marker_list) > 0:
                markers_seen_by_particle = particle.read_markers(grid)

                if len(markers_seen_by_particle) > 0:

                    marker_matched_pairs = []
                    for robot_marker in measured_marker_list:

                        if len(markers_seen_by_particle) > 0:
                            closest_part_marker = get_closest_part_marker(robot_marker, markers_seen_by_particle)
                            marker_matched_pairs.append((robot_marker, closest_part_marker))
                            markers_seen_by_particle.remove(closest_part_marker)

                    prob = 1.0
                    for marker in marker_matched_pairs:
                        distance = grid_distance(marker[0][0], marker[0][1], marker[1][0], marker[1][1])
                        angle_diff = diff_heading_deg(marker[0][2], marker[1][2])
                        dist_expression = (distance ** 2) / (2 * (MARKER_TRANS_SIGMA ** 2))
                        angle_expression = (angle_diff ** 2) / (2 * (MARKER_ROT_SIGMA ** 2))
                        prob *= math.exp(-(dist_expression + angle_expression))

                    particle_weights.append(prob)

                else:
                    particle_weights.append(0)

            else:
                particle_weights.append(1)

        else:
            particle_weights.append(0)

    sum_weights = sum(particle_weights)
    if sum_weights != 0:
        norm_part_weights = [weight / sum_weights for weight in particle_weights]
    else:
        norm_part_weights = [1 / (len(particle_weights)) for weight in particle_weights]

    random_particles = []
    random_count = 75

    measured_particles = np.random.choice(particles, size=len(particles) - random_count, replace=True, p=norm_part_weights)
    
    measured_copy = []
    for particle in measured_particles:
        measured_copy.append(Particle(particle.x, particle.y, particle.h))

    measured_particles = measured_copy

    for x in range(random_count):
        rand_x, rand_y = grid.random_free_place()
        random_particles.append(Particle(rand_x, rand_y))

    return np.concatenate([random_particles, measured_particles])

