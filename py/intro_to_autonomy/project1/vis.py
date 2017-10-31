from matplotlib import pyplot as plt

import shapely.geometry as geom
from shapely import affinity
import itertools
from matplotlib import pyplot as plt
from descartes import PolygonPatch

class Visualization:
    def __init__(self, env, start_pose):
        self.env = env
        self.ax = plot_environment(env)
        self.start_x, self.start_y = start_pose
        self.x, self.y = start_pose
        self.marker = self.ax.plot(self.start_x, self.start_y, 'bo')[0]
        plt.pause(0.01)
        
    def update_pose(self, new_pose):
        x, y = new_pose
        self.ax.plot([self.x, x], [self.y, y], color='red')
        self.marker.remove()
        self.marker = self.ax.plot(x, y, 'bo')[0]
        self.x = x
        self.y = y
        plt.pause(0.01)
    

def plot_environment(env, bounds=None, figsize=None):
    if bounds is None and env.bounds:
        minx, miny, maxx, maxy = env.bounds
    elif bounds:
        minx, miny, maxx, maxy = bounds
    else:
        minx, miny, maxx, maxy = (-10,-5,10,5)

    max_width, max_height = 12, 5.5
    if figsize is None:
        width, height = max_width, (maxy-miny)*max_width/(maxx-minx)
        if height > 5:
            width, height = (maxx-minx)*max_height/(maxy-miny), max_height
        figsize = (width, height)
    print figsize
    f = plt.figure(figsize=figsize)
    f.hold('on')
    ax = f.add_subplot(111)
    for i, obs in enumerate(env.obstacles):
        patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
        ax.add_patch(patch)
    for i, feat in enumerate(env.features):
        patch = PolygonPatch(feat, fc='green', ec='green', alpha=0.5, zorder=20)
        ax.add_patch(patch)

    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    ax.set_aspect('equal', adjustable='box')
    return ax

def plot_line(ax, line):
    x, y = line.xy
    ax.plot(x, y, color='gray', linewidth=3, solid_capstyle='round', zorder=1)


def plot_poly(ax, poly, color, alpha=1.0, zorder=1):
    patch = PolygonPatch(poly, fc=color, ec="black", alpha=alpha, zorder=zorder)
    ax.add_patch(patch)
