"""
Contains general functions to be used in the task completion algorithms.
Functions manipulating quaternions (q_mult, q_conjugate, and qv_mult) are 
from https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion.
The function get_yaw is adapted from https://stackoverflow.com/questions/56207448/efficient-quaternions-to-euler-transformation.
"""
import src.SFR as SFR
import numpy as np
import time
from src.buoys import Buoy


def seen(o, s):
    """
    Checks if the boat has seen an object before.
    Args:
        o: Buoy class object defined in the local frame.
        s: set() of Buoy class objects the boat has previously identified, defined in the global frame
    Returns:
        Boolean describing whether or not the object has been seen before. 
        Note that we consider objects with the same label identified within 1 meter of each other as the same.
    """
    for seen_o in s:
        if o.label == seen_o.label:
            global_o = np.array(map_to_global(o.x, o.y))
            if np.sum(np.square(global_o - [seen_o.x, seen_o.y])) <= 1:
                return True

    return False


def map_to_global(x, y):
    """
    Maps a local coordinate to the global frame.
    Args:
        x: x coordinate defined locally
        y: y coordinate defined locally
    Returns:
        [x, y]: a list where list[0] = globally defined x coordinate, list[1] = globally defined y coordinate
    """
    theta = SFR.heading - (np.pi/2)
    if theta < -np.pi:
        theta += 2 * np.pi
    H = np.array([[np.cos(theta), -np.sin(theta), SFR.tx],
                  [np.sin(theta), np.cos(theta), SFR.ty],
                  [0, 0, 1]])
    x, y, z = H @ np.array([x, y, 1.0])
    return [x/z, y/z]


def map_to_global_Buoy(b):
    """
    Maps a Buoy object to the global frame.
    Args:
        b: Buoy object defined in the local frame
    Returns:
        global_b: Buoy object defined in the global frame 
    """
    x_global, y_global = map_to_global(b.x, b.y)
    global_b = Buoy(b.label, x_global, y_global)
    return global_b


def get_midpoint(b1, b2):
    """
    Calculates the midpoint between two buoys in the local frame.
    Args:
        b1: competition object.
        b2: competition object.
    Returns:
        [mid_x, mid_y]: list corresponding to the local x, y coords of the midpt.
    """
    mid_x = (b1.x + b2.x) / 2
    mid_y = (b1.y + b2.y) / 2
    return [mid_x, mid_y]


def get_extended_midpoint(b1, b2, t=1):
    """
    Returns the local coordinates of a point (x, y) t meters away from the midpoint of
    b1 and b2. The line intersecting (x, y) and the midpoint is perpindicular
    to the line intersecting b1 and b2.
    Args:
        b1: competition object.
        b2: competition object.
        t: desired meters midpoint will be extended. 
        t > 0: extended closer to the boat. t < 0: extended away from the boat.
        (the point will always be perpendicular to the line of the buoys)
    Returns:
        [x, y]: list corresponding to the local x, y coords of the ext. midpt.
    """
    if b1.y != b2.y:
        ext = t/np.sqrt(1 + ((b1.x - b2.x)/(b2.y - b1.y))**2)
        if (b1.x + b2.x) < 0 or b1.y < b2.y:
            ext = -ext
        x = ext + (b1.x + b2.x)/2
        y = ((b1.x - b2.x)/(b2.y - b1.y))*ext + (b1.y + b2.y)/2
    else:
        x = (b1.x + b2.x)/2
        y = b1.y + t
    return [x, y]


def get_shifted_em(obj1, obj2, t=3):
    """
    Returns the local coordinates of a point (x, y) t meters away from object obj1.
    The line intersecting (x, y) and obj1 is perpindicular to the line intersecting
    obj1 and obj2.
    Args:
        obj1: competition object. Point returned will be in line with obj1.
        obj2: competition object.
        t: desired meters obj1 will be extended.
        t > 0: extended closer to the boat. t < 0: extended away from the boat.
        (the point will always be perpendicular to the line of the buoys)
    Returns:
        [x, y]: list corresponding to the local x, y coords of the shifted ext. midpt.
    """
    x, y = get_extended_midpoint(obj1, obj2, t)
    x, y = x + (obj1.x - obj2.x)/2, y + (obj1.y - obj2.y)/2
    return [x, y]


def get_extended_buoy(b, t=3):
    """
    Returns the local coordinates of a point (x, y) t meters away from buoy b.
    The point will lie on the line intersecting the local coordinates of b and 
    the boat. A positive t value implies extension in the direction toward the origin 
    and a negative t value implies extension in the direction away from the buoy. 
        b: buoy object.
        t: desired meters away from the buoy.
    Returns:
        [x, y]: list corresponding to the local coordinates of the point extended t from b
    """
    ext = t/(np.sqrt(b.x**2 + b.y**2))
    x = (1 - ext) * b.x
    y = (1 - ext) * b.y
    return [x, y]


def filter_objects(labels, previously_seen=[],  axis='y'):
    """
    Filter objects array to contain only objects with a label in labels, sorts 
    the objects by specified axis, and keeps track of objects already seen.
    Args:
        labels: list of strings.
        previously_seen: set.
        axis: one of 'x', 'y', or 'z'.
    Returns:
        objs: sorted numpy array of objects.
        seen: the updated set of objects seen.
    """

    # objs only has objects with a label in labels
    objs = np.array(
        list(filter(lambda o: not (seen(o, previously_seen)) and o.label in labels, SFR.objects)))

    # if we see no objects that fit criteria return empty set
    if not np.all(objs):
        return np.array([]), previously_seen

    # sort objects by axis
    if len(objs) > 0:
        def get_attr(o): return getattr(o, axis)
        get_axis_vals = np.vectorize(get_attr)
        objs = objs[get_axis_vals(objs).argsort()]

    s = np.concatenate((previously_seen, objs))

    return objs, s


def pivot_to_gate(s):
    """
    Pivots the boat in place to identify a gate of buoys. Will not turn more than
    90 degrees (pi/2) radians in either direction.
    Args:
        seen: set of buoys seen
    Returns:
        gate: numpy array [red buoy object, green buoy object] corresponding to the
        closest gate found. An entry will be None if no matching object was found.
    """
    # [red buoy, green buoy]
    gate = np.array([[None, None]])
    start = time.time()

    # Search left then search right. Conditions: a gate buoy has not yet been
    # identified and we haven't turned 90 degrees (pi/2 radians)
    while time.time() - start < np.pi / 2:
        buoys, s = SFR.filter_buoys(["green_buoy", "red_buoy"], s)
        for b in buoys:
            if b.label == "red_buoy" and (not gate[0] or gate[0].y > b.y):
                gate[0] = b
            elif not gate[1] or gate[1].y > b.y:
                gate[1] = b
        # pivot left

    start = time.time()
    while time.time() - start < np.pi:
        buoys, s = SFR.filter_buoys(["green_buoy", "red_buoy"], s)
        for b in buoys:
            if b.label == "red_buoy" and (not gate[0] or gate[0].y > b.y):
                gate[0] = b
            elif not gate[1] or gate[1].y > b.y:
                gate[1] = b
        # pivot right

    # pivot to recenter
    start = time.time()
    while time.time() - start < np.pi/2:
        pass
        # pivot left

    return gate, s


def filter_correct_sign(previously_seen=[]):
    """
    Filter objects array to contain only the sign with the correct color.
    Args:
        previously_seen: set.
    Returns:
        objs: sorted numpy array of correct sign
        seen: seen objects
    """
    signs = np.array(
        list(filter(lambda o: not (seen(o, previously_seen)) and o.label[0] == SFR.sign_color and o.label[-5:] == "thing", SFR.objects)))
    s = previously_seen + signs

    if signs.size == 1:
        return signs, s
    else:
        return np.array([]), previously_seen


def filter_signs(previously_seen=[]):
    """
    Filter objects array to contain only objects with a label in labels, sorts 
    the objects by specified axis, and keeps track of objects already seen.
    Args:
        previously_seen: set.
    Returns:
        objs: sorted numpy array of signs
        seen: seen objects
    """

    signs = np.array(
        list(filter(lambda o: not (seen(o, previously_seen)) and o.label[-4:] == "sign", SFR.objects)))

    # sort objects by axis
    if len(signs) > 0:
        def get_attr(o): return getattr(o, 'x')
        get_axis_vals = np.vectorize(get_attr)
        signs = signs[get_axis_vals(signs).argsort()]

    s = np.concatenate((previously_seen, signs))

    if signs.size > 1:
        return signs, s
    else:
        return np.array([]), previously_seen
