
import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

import env, utils
# import plotting


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.cost = np.inf


class FMTN:
    def __init__(self, x0, x1, x2, x3, x4, x5, x6, x7, x9, search_radius):
        self.x_goal1 = Node(x0)
        self.x_goal2 = Node(x1)
        self.x_goal3 = Node(x2)
        self.x_goal4 = Node(x3)
        self.x_goal5 = Node(x4)
        self.x_goal6 = Node(x5)
        self.x_goal7 = Node(x6)
        self.x_goal8 = Node(x7)
        self.x_goal9 = Node(x9)

        self.search_radius = search_radius

        self.env = env.Env()
        # self.plotting = plotting.Plotting(x0, x1)
        self.utils = utils.Utils()

        self.fig, self.ax = plt.subplots()
        self.delta = self.utils.delta
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.sample_numbers = 10000
        self.V = set()
        self.V_unvisited = set()
        self.V_open = set()
        self.V_closed = set()


    def Setup(self):
        self.V = set()
        self.V_unvisited = set()
        self.V_open = set()
        self.V_closed = set()
        samples = self.SampleFree()
        self.V.update(samples)
        self.V_unvisited.update(samples)
        self.n = self.sample_numbers
        self.rn = self.search_radius * math.sqrt((math.log(self.n) / self.n))
        self.plot_grid("Fast Marching For Multi-Sensors Network")



    def plot(self,x_start,x_end):

        self.x_start = x_start
        self.x_end = x_end
        self.x_start.cost = 1
        self.V.add(self.x_start)
        self.V_unvisited.add(self.x_end)
        self.V_open.add(self.x_start)
        z = self.x_start

        Visited = []

        while z is not self.x_end:
            V_open_new = set()
            X_near = self.Near(self.V_unvisited, z, self.rn)
            Visited.append(z)

            for x in X_near:
                Y_near = self.Near(self.V_open, x, self.rn)
                cost_list = {y: y.cost + self.Cost(y, x) for y in Y_near}
                y_min = min(cost_list, key=cost_list.get)

                if not self.utils.is_collision(y_min, x):
                    x.parent = y_min
                    V_open_new.add(x)
                    self.V_unvisited.remove(x)
                    x.cost = y_min.cost + self.Cost(y_min, x)

            self.V_open.update(V_open_new)
            self.V_open.remove(z)
            self.V_closed.add(z)

            if not self.V_open:
                print("open set empty!")
                break

            cost_open = {y: y.cost for y in self.V_open}
            z = min(cost_open, key=cost_open.get)

        path_x1 = []
        path_y1 = []
        node1 = self.x_end
        while node1.parent:
            path_x1.append(node1.x)
            path_y1.append(node1.y)
            node1 = node1.parent

        path_x1.append(self.x_start.x)
        path_y1.append(self.x_start.y)

        # self.animation(path_x0, path_y0, Visited_0[1: len(Visited_0)])
        # for node in self.V:
        #     plt.plot(node.x,node.y,marker='.', color='lightgrey', markersize=3)

        count1 = 0
        for node1 in Visited[1:len(Visited)]:
            count1 += 1
            plt.plot([node1.x, node1.parent.x], [node1.y, node1.parent.y], '-g')
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            if count1 % 10 == 0:
                plt.pause(0.001)

        plt.plot(path_x1, path_y1,linewidth = 3, color = 'red')
        plt.pause(0.001)
        return None




    def Cost(self, x_start, x_end):
        if self.utils.is_collision(x_start, x_end):
            return np.inf
        else:
            return self.calc_dist(x_start, x_end)

    @staticmethod
    def calc_dist(x_start, x_end):
        return math.hypot(x_start.x - x_end.x, x_start.y - x_end.y)

    @staticmethod
    def Near(nodelist, z, rn):
        return {nd for nd in nodelist
                if 0 < (nd.x - z.x) ** 2 + (nd.y - z.y) ** 2 <= rn ** 2}

    def SampleFree(self):
        n = self.sample_numbers
        delta = self.utils.delta
        Sample = set()

        ind = 0
        while ind < n:
            node = Node((random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))
            if self.utils.is_inside_obs(node):
                continue
            else:
                Sample.add(node)
                ind += 1

        return Sample


    def plot_grid(self, name):

        plt.plot(self.x_goal1.x, self.x_goal1.y, "rs", linewidth=3)
        plt.plot(self.x_goal2.x, self.x_goal2.y, "rs", linewidth=3)
        plt.plot(self.x_goal2.x, self.x_goal2.y, "rs", linewidth=3)
        plt.plot(self.x_goal3.x, self.x_goal3.y, "rs", linewidth=3)
        plt.plot(self.x_goal4.x, self.x_goal4.y, "rs", linewidth=3)
        plt.plot(self.x_goal5.x, self.x_goal5.y, "rs", linewidth=3)
        plt.plot(self.x_goal6.x, self.x_goal6.y, "rs", linewidth=3)
        plt.plot(self.x_goal7.x, self.x_goal7.y, "rs", linewidth=3)
        plt.plot(self.x_goal8.x, self.x_goal8.y, "rs", linewidth=3)
        plt.title(name)
        plt.axis("equal")


    def main(self):
        self.Setup()
        self.plot(x_start=self.x_goal4, x_end=self.x_goal3)

        self.Setup()
        self.plot(x_start=self.x_goal2, x_end=self.x_goal3)

        self.Setup()
        self.plot(x_start=self.x_goal5, x_end=self.x_goal4)

        self.Setup()
        self.plot(x_start=self.x_goal6, x_end=self.x_goal7)

        self.Setup()
        self.plot(x_start=self.x_goal6, x_end=self.x_goal5)

        self.Setup()
        self.plot(x_start=self.x_goal8, x_end=self.x_goal9)

        self.Setup()
        self.plot(x_start=self.x_goal8, x_end=self.x_goal7)

        self.Setup()
        self.plot(x_start=self.x_goal1, x_end=self.x_goal2)
        #plt.plot(x1[0],x1[1],linewidth = 2, color = 'red')
        #plt.pause(0.001)

        plt.show()