from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.patches import Ellipse
import time as time_library
from IPython import display

"""
Modified from the Incremental Path Planning pset Grid class :)
"""

class WrongGridFormat(Exception):
    def __init__(self):
        pass
    def __str__(self):
        return "Wrong grid format. Use 0 for free space, 1 for obstacle, S and G for start and goal, respectively, and R for robot."

class WrongCellType(Exception):
    def __init__(self):
        pass

    def __str__(self):
        return "Invalid cell type. A cell type should be one of: " + repr(VALID_CELL_TYPES)

PREFERRED_MAX_FIG_WIDTH = 12
PREFERRED_MAX_FIG_HEIGHT = 8

# Easily refer to cell-types
CELL_FREE = 0
CELL_OBSTACLE = 1
CELL_START = 7
CELL_GOAL = 9
VALID_CELL_TYPES = [CELL_FREE, CELL_OBSTACLE, CELL_START, CELL_GOAL]

COLOR = { \
    "free": "white", \
    "obstacle": "#333333", \
    "new-obstacle": "None", \
    "robot": "black", \
    "start": "#00DD44", \
    "goal": "red", \
    "path-travelled": "green", \
    "path-future": "#DD0000"
    }

class GridMDP(object):
    def __init__(self, num_cols=10, num_rows=10, figsize=None):
        self.num_cols = num_cols
        self.num_rows = num_rows
        
        self.generate_grid(num_cols, num_rows)

        self.xlimits = (minx, maxx) = (0, num_cols)
        self.ylimits = (miny, maxy) = (0, num_rows)
        self.cell_size = (maxx-minx) / num_cols, (maxy-miny) / num_rows

        if figsize:
            self.figsize = figsize
        else:
            width, height = maxx - minx, maxy - miny
            if width > height:
                self.figsize = (PREFERRED_MAX_FIG_WIDTH, height * PREFERRED_MAX_FIG_WIDTH / width)
            else:
                self.figsize = (width * PREFERRED_MAX_FIG_HEIGHT / height, PREFERRED_MAX_FIG_HEIGHT)

    def __eq__(self, other):
        return self.xlimits == other.xlimits and self.ylimits == other.ylimits \
            and self.grid_array == other.grid_array

    def clone_template(self):
        num_cols, num_rows = self.size
        new_grid = GridMDP(num_cols, num_rows, self.figsize)
        return new_grid

    def copy(self):
        new_grid = self.clone_template()
        new_grid.grid_array = self.get_grid_array().copy()
        return new_grid

    @property
    def size(self):
        return self.grid_array.shape

    def get_grid_array(self):
        return self.grid_array

    def get_cell(self, x, y):
        return self.grid_array[x, y]

    def set_cell(self, x, y, val):
        self.grid_array[x, y] = val
        return True

    def generate_grid(self, num_cols, num_rows):
        self.grid_array = np.zeros([num_cols, num_rows])
        self.rewards = np.zeros([num_cols, num_rows])
        
    def mark_cell_as(self, x, y, what_type):
        if what_type not in VALID_CELL_TYPES:
            raise WrongCellType

        self.grid_array[x, y] = what_type

    def get_cells_of_type(self, what_type):
        if what_type not in VALID_CELL_TYPES:
            raise WrongCellType

        return zip(*np.where(self.grid_array == what_type))

    def get_cell_reward(self, x, y):
        return self.rewards[x, y]
    
    def set_cell_reward(self, x, y, reward):
        self.rewards[x, y] = reward
        return True

    def clear(self):
        self.grid_array = np.zeros([self.num_cols, self.num_rows])
        self.rewards = np.zeros([self.num_cols, self.num_rows])

    def cell_center(self, ix, iy):
        """Returns the center xy point of the cell."""
        minx, maxx = self.xlimits
        miny, maxy = self.ylimits
        cwidth, cheight = self.cell_size
        return minx + (ix+0.5) * cwidth, miny + (iy+0.5) * cheight

    def _cell_vertices(self, ix, iy):
        cwidth, cheight = self.cell_size
        x, y = self.cell_center(ix, iy)
        verts = [(x + ofx*0.5*cwidth, y + ofy*0.5*cheight) for ofx, ofy in [(-1,-1),(-1,1),(1,1),(1,-1)]]
        return verts

    def export_to_dict(self):
        export_dict = {}
        export_dict['grid'] = self.grid_array.tolist()
        return export_dict

    def load_from_dict(self, grid_dict):
        self.grid_array = np.array(grid_dict['grid'])

    def get_goal(self):
        return self.get_cells_of_type(CELL_GOAL)[0]

    # DRAWING METHODS

    def draw(self):
        cols, rows = self.size
        minx, maxx = self.xlimits
        miny, maxy = self.ylimits

        cwidth, cheight = self.cell_size

        x = map(lambda i: minx + cwidth*i, range(cols+1))
        y = map(lambda i: miny + cheight*i, range(rows+1))

        f = plt.figure(figsize=self.figsize)
        hlines = np.column_stack(np.broadcast_arrays(x[0], y, x[-1], y))
        vlines = np.column_stack(np.broadcast_arrays(x, y[0], x, y[-1]))
        lines = np.concatenate([hlines, vlines]).reshape(-1, 2, 2)
        line_collection = LineCollection(lines, color="black", linewidths=0.5)
        ax = plt.gca()
        ax.add_collection(line_collection)
        ax.set_xlim(x[0]-1, x[-1]+1)
        ax.set_ylim(y[0]-1, y[-1]+1)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis('off')
        self._draw_obstacles(plt.gca())
        self._draw_start_goal(plt.gca())
        self._draw_reward_values(plt.gca())
        self.axes = plt.gca()
        return plt.gca()

    def _draw_obstacles(self, axes):
        verts = [self._cell_vertices(ix, iy) for ix,iy in self.get_cells_of_type(CELL_OBSTACLE)]
        collection_recs = PolyCollection(verts, facecolors=COLOR["obstacle"])
        axes.add_collection(collection_recs)

    def _draw_start_goal(self, axes):
        start_verts = [self._cell_vertices(ix, iy) for ix,iy in self.get_cells_of_type(CELL_START)]
        goal_verts = [self._cell_vertices(ix, iy) for ix,iy in self.get_cells_of_type(CELL_GOAL)]
        collection_recs = PolyCollection(start_verts, facecolors=COLOR["start"])
        axes.add_collection(collection_recs)
        collection_recs = PolyCollection(goal_verts, facecolors=COLOR["goal"])
        axes.add_collection(collection_recs)

    def _draw_reward_values(self, axes):
        for i in range(self.num_cols):
            for j in range(self.num_rows):
                center = self.cell_center(i, j)
                reward = self.get_cell_reward(i, j)
                if reward != 0:
                    axes.text(center[0] - 0.1, center[1], str(reward), fontsize=12)
  
    def draw_policy(self, pi):
         chars = {'(1,0)':'>', '(0,1)':'^', '(-1,0)':'<', '(0,-1)':'v', None: '.'}
         for (s, action) in pi.iteritems():
             center = self.cell_center(int(s[1]), int(s[3]))
             self.axes.text(center[0], center[1]-0.2, chars[action], fontsize=12)
  