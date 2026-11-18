# SPDX-License-Identifier: MulanPSL-2.0
"""SE(2) pose transport across the localization-to-mapping process swap."""
import math


def compose(a, b):
    x, y, yaw = a
    c, s = math.cos(yaw), math.sin(yaw)
    return (x+c*b[0]-s*b[1], y+s*b[0]+c*b[1], math.atan2(math.sin(yaw+b[2]), math.cos(yaw+b[2])))


def inverse(p):
    x, y, yaw = p
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c*x-s*y, s*x-c*y, -yaw)


def advance(map_pose, anchor_odom, current_odom):
    return compose(compose(map_pose, inverse(anchor_odom)), current_odom)


def tuple_from_transform(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    return (t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
