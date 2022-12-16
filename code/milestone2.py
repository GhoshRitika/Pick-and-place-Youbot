import modern_robotics as mr
import numpy as np

def TrajectoryGenerator(Tseinit, Tscint, Tscfinal, Tcegrasp, Tcestand, k=1):
    """
    This function calculates the trajectory of end effector picking and placing a box on a plane.
    This trajectory is divided into 8 segments, each segment performing a part of the task, like
    going from home configuration to the x and y coordinates of the box.
    Args:
    Tseint: The initial configuration of the end-effector in the reference trajectory.
    Tscint: The cube's initial configuration.
    Tscfinal: The cube's desired final configuration.
    Tcegrasp: The end-effector's configuration relative to the cube when it is grasping the cube.
    Tcestand: The end-effector's standoff configuration above the cube, before and after grasping,
            relative to the cube. This specifies the configuration of the end-effector {e} relative
            to the cube frame {c} before lowering to the grasp configuration Tce,grasp, for example.
    k: The number of trajectory reference configurations per 0.01 seconds. The value k is an integer
        with a value of 1 or greater.
    Return:
    Traj: A representation of the N configurations of the end-effector along the entire concatenated
         eight-segment reference trajectory. Each of these N reference points represents a
         transformation matrix Tse of the end-effector frame {e} relative to {s} at an instant in
         time, plus the gripper state (0 or 1).
    A csv file with the entire eight-segment reference trajectory. Each line of the csv file
    corresponds to one configuration Tse of the end-effector, expressed as 13 variables separated
    by commas. The 13 variables are, in order
    """
    #total no. of transformation matrices expected
    N = int((3*6 + 1.3)*k /0.01)
    traj = np.zeros([N, 13])
    #gripper state
    gs = 0
    count = 0
    #Eah transformation matrix of the 8 segments in order of execution
    T = [Tseinit, Tscint@Tcestand, Tscint@Tcegrasp, Tscint@Tcegrasp,Tscint@Tcestand, 
        Tscfinal@Tcestand, Tscfinal@Tcegrasp, Tscfinal@Tcegrasp, Tscfinal@Tcestand]
    for i in range(8):
        traj1 = []
        #The segment that grasps the box
        if i == 2:
            gs = 1
            n = 0.65*k/0.01
            traj1 = mr.ScrewTrajectory(T[i],T[i+1], 0.65, n, 5)
        #Segement of trajectory that releases grasp on the box
        elif i == 6:
            gs = 0
            n = 0.65*k/0.01
            traj1 = mr.ScrewTrajectory(T[i],T[i+1], 0.65, n, 5)
        else:
            n = 3*k/0.01
            traj1 = mr.ScrewTrajectory(T[i],T[i+1], 3, n, 5)
        for j in range(len(traj1)):
            x = traj1[j]
            #Retrieving the 13 elements in the right order
            traj[count, 0] = x[0,0]
            traj[count, 1] = x[0,1]
            traj[count, 2] = x[0,2]
            traj[count, 3] = x[1,0]
            traj[count, 4] = x[1,1]
            traj[count, 5] = x[1,2]
            traj[count, 6] = x[2,0]
            traj[count, 7] = x[2,1]
            traj[count, 8] = x[2,2]
            traj[count, 9] = x[0,3]
            traj[count, 10] = x[1,3]
            traj[count, 11] = x[2,3]
            traj[count, 12] = gs
            count += 1

    return traj

def main():
    Tsci = np.array([[1, 0, 0, 1],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])
    Tscf = np.array([[0, 1, 0, 0],
                    [-1, 0, 0, -1],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])
    #Rotation about Y axis by 2pi/4 angle
    Tstandoff = np.array([[-0.707, 0, 0.707, 0],
                        [0, 1, 0, 0],
                        [-0.707, 0, -0.707, 0.5],
                        [0, 0, 0, 1]])
    Tinit = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0, 1]])
    #Rotation about Y axis by 2pi/4 angle
    Tgrasp = np.array([[-0.707, 0, 0.707, 0],
                        [0, 1, 0, 0],
                        [-0.707, 0, -0.707, 0],
                        [0, 0, 0, 1]])
    mytraj = TrajectoryGenerator(Tinit, Tsci, Tscf, Tgrasp, Tstandoff, 1)
    np.savetxt("Ghosh_Ritika_milestone2.csv", mytraj, delimiter = ",")

if __name__ == "__main__":
    main()