"""
Contains general functions to be used in the task completion algorithms.
Functions manipulating quaternions (q_mult, q_conjugate, and qv_mult) are 
from https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion.
The function get_yaw is adapted from https://stackoverflow.com/questions/56207448/efficient-quaternions-to-euler-transformation.
"""
import src.SFR as SFR
import numpy as np
import time


def seen(o, s):
    for seen_o in s:
        if o.label == seen_o.label:
            global_o = np.array(map_to_global(o.x, o.z))
            global_seen_o = np.array(map_to_global(seen_o.x, seen_o.z))
            if np.sum(np.square(global_o - global_seen_o)) <= 1:
                return True

    return False


def q_mult(q1, q2):
    # Computes the product of two quaternions.
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z


def q_conjugate(q):
    # Computes the conjugate of a quaternion.
    w, x, y, z = q
    return w, -x, -y, -z


def qv_mult(q1, v1):
    # Computes the product of a quaternion.
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))


def map_to_global(x, z):
    """
    Maps a local coordinate to the global frame.
    """
    q1 = (SFR.ow, SFR.ox, SFR.oy, SFR.oz)
    v1 = (x, 0, z)
    # rotation
    _, rx, _, rz = qv_mult(q1, v1)
    # translation
    rtx, rtz = rx + SFR.tx, rz + SFR.tz
    return [rtx, rtz]


def get_midpoint(b1, b2):
    """
    Calculates the midpoint between two buoys in the local frame.
    Args:
        b1: competition object.
        b2: competition object.
    Returns:
        [mid_x, mid_z]: list corresponding to the local x, z coords of the midpt.
    """
    mid_x = (b1.x + b2.x) / 2
    mid_z = (b1.z + b2.z) / 2
    return [mid_x, mid_z]


def get_extended_midpoint(b1, b2, t=1):
    """
    Returns the local coordinates of a point (x, z) t meters away from the midpoint of
    b1 and b2. The line intersecting (x, z) and the midpoint is perpindicular
    to the line intersecting b1 and b2.
    Args:
        b1: competition object.
        b2: competition object.
        t: desired meters midpoint will be extended. 
        t > 0: extended away from boat. t < 0: extended closer to boat.
    Returns:
        [x, z]: list corresponding to the local x, z coords of the ext. midpt.
    """
    if b1.z != b2.z:
        ext = t/np.sqrt(1 + ((b1.x - b2.x)/(b2.z - b1.z))**2)
        if (b1.x + b2.x) < 0 or b1.z < b2.z:
            ext = -ext
        x = ext + (b1.x + b2.x)/2
        z = ((b1.x - b2.x)/(b2.z - b1.z))*ext + (b1.z + b2.z)/2
    else:
        x = (b1.x + b2.x)/2
        z = b1.z + t
    return [x, z]


def get_shifted_em(obj1, obj2, t=3):
    """
    Returns the local coordinates of a point (x, z) t meters away from object obj1.
    The line intersecting (x, z) and obj1 is perpindicular to the line intersecting
    obj1 and obj2.
    Args:
        obj1: competition object. Point returned will be in line with obj1.
        obj2: competition object.
        t: desired meters obj1 will be extended.
    Returns:
        [x, z]: list corresponding to the local x, z coords of the shifted ext. midpt.
    """
    x, z = get_extended_midpoint(obj1, obj2, t)
    x, z = x + (obj1.x - obj2.x)/2, z + (obj1.z - obj2.z)/2
    return [x, z]


def get_extended_buoy(b, t=3):
    """
    Returns the local coordinates of a point (x, z) t meters away from buoy b.
    The point will lie on the line intersecting the local coordinates of b and 
    the boat.
    Args:
        b: buoy object.
        t: desired meters away from the buoy.
    Returns:
        [x, z]: list corresponding to the local coordinates of the point extended t from b
    """
    ext = t/np.sqrt(b.x**2 + b.z**2)
    x = (1 - ext) * b.x
    z = (1 - ext) * b.z
    return [x, z]


def get_yaw():
    """
    Uses the orientation quaternion to calculate yaw (rotation in radians around
    the Y axis). 
    """
    w, x, y, z = SFR.ow, SFR.ox, SFR.oy, SFR.oz

    t0 = 2*(w * y + x * z)
    t1 = w**2 + x**2 - z**2 - y**2

    # ranges from -pi to pi
    yaw = np.arctan2(t0, t1)

    return yaw


def filter_objects(labels, previously_seen=[],  axis='z'):
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
        list(filter(lambda o: not(seen(o, previously_seen)) and o.label in labels, SFR.objects)))

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
    thruster_utils.sendValue(1500, 1500)
    while time.time() - start < np.pi / 2:
        buoys, s = SFR.filter_buoys(["green_buoy", "red_buoy"], s)
        for b in buoys:
            if b.label == "red_buoy" and (not gate[0] or gate[0].z > b.z):
                gate[0] = b
            elif not gate[1] or gate[1].z > b.z:
                gate[1] = b
        # pivot left

    start = time.time()
    while time.time() - start < np.pi:
        buoys, s = SFR.filter_buoys(["green_buoy", "red_buoy"], s)
        for b in buoys:
            if b.label == "red_buoy" and (not gate[0] or gate[0].z > b.z):
                gate[0] = b
            elif not gate[1] or gate[1].z > b.z:
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
        list(filter(lambda o: not(seen(o, previously_seen)) and o.label[0] == SFR.sign_color and o.label[3:] == "sign", SFR.objects)))
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
        list(filter(lambda o: not(seen(o, previously_seen)) and o.label[3:] == "sign", SFR.objects)))

    # sort objects by axis
    if len(signs) > 0:
        def get_attr(o): return getattr(o, 'x')
        get_axis_vals = np.vectorize(get_attr)
        signs = signs[get_axis_vals(signs).argsort()]

    s = previously_seen + signs

    if signs.size > 1:
        return signs, s
    else:
        return np.array([]), previously_seen