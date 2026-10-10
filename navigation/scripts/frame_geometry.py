"""Planar frame conversion; transform means target <- source."""
import math


def wrap(value):return math.atan2(math.sin(value),math.cos(value))


def apply(transform,point):
    tx,ty,angle=transform;c,s=math.cos(angle),math.sin(angle)
    return (tx+c*point[0]-s*point[1],ty+s*point[0]+c*point[1])


def inverse(transform):
    tx,ty,angle=transform;c,s=math.cos(angle),math.sin(angle)
    return (-c*tx-s*ty,s*tx-c*ty,-angle)
