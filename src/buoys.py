class Buoy:
    """
    Class for buoy objects.
    """
    label = ""
    # indicates the local x, y, and z vector values. Indicate object's position from the ZED camera
    x, y, z = 0.0, 0.0, 0.0

    def __init__(self, label, x, y, z):
        self.label = label
        self.x = x
        self.y = y
        self.z = z
