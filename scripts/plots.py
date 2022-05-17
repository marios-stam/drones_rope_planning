#!/usr/bin/env python3
# print working directory

#import plt
import matplotlib.pyplot as plt

if __name__ == "__main__":
    planner_names = ["RRT",
                     "RRT*",
                     "RRTConnect",
                     "InformedRRT*",
                     "PRM",
                     "KPIECE"]

    planner_times = [1135, 3305, 2205, 3128, 18468, 21375]

    obstacles_names = ["corridor", "hole-inclined", "tunnel", "hole", "wall"]
    obstacles_times = [2200, 75, 6973, 15000, 1983]

    # bar chart
    plt.figure()
    plt.bar(planner_names, planner_times)
    plt.xlabel("Planner")
    plt.ylabel("Time (msec)")

    plt.figure()
    plt.bar(obstacles_names, obstacles_times)
    plt.xlabel("Obstacle")
    plt.ylabel("Time (msec)")

    plt.show()
