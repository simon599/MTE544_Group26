# used to vectorize point operations in the trajectory planner
import numpy as np

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

# Type of trajectory
PARABOLA_TRAJECTORY='parabola'; SIGMOID_TRAJECTORY='sigmoid'

# Defines number of waypoints in specified trajectory
# Needs to be tuned in the lab
N_POINTS = 15

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0], trajectory=PARABOLA_TRAJECTORY):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner(trajectory)


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, trajectory):
        
        if trajectory == PARABOLA_TRAJECTORY:
            x_vals = np.linspace(0, 1.5, N_POINTS)
            y_vals = np.power(x_vals, 2)

        elif trajectory == SIGMOID_TRAJECTORY:
            x_vals = np.linspace(0, 2.5, N_POINTS)
            y_vals = 2 / (1 + np.exp(- 2 * x_vals))

        else:
            print("Error: unrecognized trajectory type.")
            # remain in current position
            x_vals = [0]
            y_vals = [0]

        # the return should be a list of trajectory points: [[x1,y1], ..., [xn,yn]]
        return np.column_stack([x_vals, y_vals]).tolist()

