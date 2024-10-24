import numpy as np

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

# global variable to set trajectory type
trajectory_type = "parabola"


class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        # define step size between x coordinates (in m)
        step_size = 1e-1
        if trajectory_type == "parabola":
            x_vals = np.arange(0, 1.5, step_size)
            y_vals = np.power(x_vals, 2)
        elif trajectory_type == "sigmoid":
            x_vals = np.arange(0, 2.5, step_size)
            y_vals = 2 / (1 + np.exp(- 2 * x_vals))
        else:
            print("Error: unrecognized trajectory type.")
            x_vals = [0]
            y_vals = [0]

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        return list(zip(x_vals, y_vals))

