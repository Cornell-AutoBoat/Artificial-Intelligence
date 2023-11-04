from a_star import get_waypoints

# TODO: make an actual test file with pytest

maze = [[0, 0, 1, 1, 1],
        [0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0]]

start = (0, 0)
end = (4, 4)

path = get_waypoints(maze, start, end, allow_diagonal_movement=True)

print(path)

# test_simulate_magellans_route():
maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1],
        [0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1]]

start = (4, 0)
end = (5, 14)

path = get_waypoints(maze, start, end, allow_diagonal_movement=True)
# potential issue with "giving up on pathfinding too many iterations"
# it seems to go backwards sometimes for some reason

print(path)
