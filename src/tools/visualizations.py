import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from src import SFR
from src.tools.utils import map_to_global_Buoy

def add_line(path):
    for i in range(0, len(path)):
        plt.plot(path[i][0], path[i][1], '.', color='coral', markersize=10)

    for i in range(0, len(path)-1):
        plt.plot([path[i][0], path[i+1][0]],
                 [path[i][1], path[i+1][1]], color='b')

    plt.axis('scaled')
    plt.axhline(y=0, color='r', linestyle='-')
    #plt.ylabel('height')
    plt.show()


def add_complicated_line(path, lineStyle, lineColor, lineLabel):
    for i in range(0, len(path)):
        plt.plot(path[i][0], path[i][1], '.', color='coral', markersize=10)

    for i in range(0, len(path)-1):
        if(i == 0):
            # plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],color='b')
            plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1]
                                                  [1]], lineStyle, color=lineColor, label=lineLabel)
        else:
            plt.plot([path[i][0], path[i+1][0]], [path[i][1],
                                                  path[i+1][1]], lineStyle, color=lineColor)

    plt.axis('scaled')


def highlight_points(points, pointColor):
    for point in points:
        plt.plot(point[0], point[1], '.', color=pointColor, markersize=10)


def draw_circle(x, y, r, circleColor):
    xs = []
    ys = []
    angles = np.arange(0, 2.2*np.pi, 0.5)

    for angle in angles:
        xs.append(r*np.cos(angle) + x)
        ys.append(r*np.sin(angle) + y)

    plt.plot(xs, ys, '-', color=circleColor)
    plt.axis('scaled')

def path_visualizer(orig_path, new_path, fig_size, field_size, fname):

    field = plt.figure()
    xscale, yscale = fig_size
    path_ax = field.add_axes([0, 0, xscale, yscale])
    add_complicated_line(orig_path, '--', 'grey', 'original')
    add_complicated_line(new_path, '--', 'orange', 'smoothed')

    path_ax.plot(orig_path[0][0], orig_path[0][1], "x", color="magenta")
    path_ax.plot(orig_path[1][0], orig_path[1][1], "x", color="magenta")

    xMin, yMin, xMax, yMax = field_size

    # plotting different buoys from the buoy list 
    for obj in SFR.objects:
        global_obj = map_to_global_Buoy(obj)
        c = global_obj.label[0]
        path_ax.plot(global_obj.x, global_obj.y, '.', color=c, markersize=10)

    # plot field
    path_ax.plot([xMin, xMax], [yMin, yMin], color='black')
    path_ax.plot([xMin, xMin], [yMin, yMax], color='black')
    path_ax.plot([xMax, xMax], [yMin, yMax], color='black')
    path_ax.plot([xMax, xMin], [yMax, yMax], color='black')
    # set grid
    xTicks = np.arange(xMin, xMax+1, 2)
    yTicks = np.arange(yMin, yMax+1, 2)

    path_ax.set_xticks(xTicks)
    path_ax.set_yticks(yTicks)
    path_ax.grid(True)

    path_ax.set_xlim(xMin-0.25, xMax+0.25)
    path_ax.set_ylim(yMin-0.25, yMax+0.25)

    # plot start and end
    path_ax.plot(new_path[0][0], new_path[0][1], 's',
                 color='navy', markersize=10, label='start')
    path_ax.plot(new_path[-1][0], new_path[-1][1], 'X',
                 color='gold', markersize=10, label='end')
    path_ax.legend(loc='upper left')


    plt.savefig(fname=fname,bbox_inches='tight', pad_inches= 1.5)