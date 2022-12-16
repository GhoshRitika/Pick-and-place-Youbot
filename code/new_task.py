"""Run 'python3 code/new_task.py' from the Ghosh_Ritika_capstone directory. It will automatically save
csv files, log file and plots in the Results/NewTask directory.
"""
import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt
from milestone1 import NextState
from milestone2 import TrajectoryGenerator
from milestone3 import FeedbackControl
import logging

def main():
    logging.basicConfig(filename ='Results/NewTask/runscript.log', level = logging.INFO)
    #The actual initial configuration of the youBot (13-vector)
    # chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state

    #this is the default configuration
    # init_config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    #Uncomment to get initial position and orientation error
    init_config = np.array([0.523599, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]) 

    # the initial configuration of the end effector in the reference trajectory
    Tse_init = np.array([[ 0, 0, 1,   0], 
                         [ 0, 1, 0,   0], 
                         [-1, 0, 0, 0.5],
                         [ 0, 0, 0,   1]])
    # the cube's initial configuration (x, y, theta) = (1.5, 1.0, 0)
    Tsc_init = np.array([[1, 0, 0,     1.5],
                         [0, 1, 0,     1.0],
                         [0, 0, 1,   0.025],
                         [0, 0, 0,     1]])
    # the cube's desired final configuration (x, y, theta) = (0, -1.5, -pi/2)
    Tsc_final = np.array([[ 0, 1, 0,     0],
                          [-1, 0, 0,  -1.5],
                          [ 0, 0, 1, 0.025],
                          [ 0, 0, 0,     1]])
    # the end effector's configuration relative to the cube when grasping
    #Rotation about Y axis by 2pi/4 angle
    Tce_grasp = np.array([[-0.707, 0,  0.707, 0],
                          [     0, 1,      0, 0],
                          [ -0.707, 0, -0.707, 0],
                          [     0, 0,      0, 1]])

    # the end effector's standoff above the cube, before and after grasping
    #Rotation about Y axis by 2pi/4 angle
    Tce_standoff = np.array([[-0.707,  0,  0.707,   0],
                             [     0,  1,       0,   0],
                             [-0.707,  0, -0.707, 0.15],
                             [     0,  0,      0,    1]])

    # the fixed offset from the chassis frame {b} to the base frame of the arm {0}
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,      1]])

    # end-effector frame {e} relative to the arm base frame {0}
    M0 = np.array([[1, 0, 0,  0.033],
                   [0, 1, 0,      0],
                   [0, 0, 1, 0.6546],
                   [0, 0, 0,      1]])

    # the screw axes for the five joints in the end-effector frame {e}
    Blist = np.array([[0,  0, 1,       0, 0.033, 0],
                      [0, -1, 0, -0.5076,     0, 0],
                      [0, -1, 0, -0.3526,     0, 0],
                      [0, -1, 0, -0.2176,     0, 0],
                      [0,  0, 1,       0,     0, 0]]).T

    # feedback control, Kp and Ki
    #The best values
    Kp_gain = 12
    Kp = np.identity(6) * Kp_gain
    Ki_gain = 0.5
    Ki = np.identity(6) * Ki_gain

    k = 1
    max_jtspeed = 2.5 
    max_wheelspeed = 15 
    dt = 0.01
    t = 20             # total time (sec)
    logging.info('Calling TrajectoryGenerator function')
    traj_list = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)

    N = len(traj_list)  # number of iterations
    config_list = np.zeros((N, 13))
    Xerr_plot = np.zeros((N, 6))
    config_list[0] = init_config
    current_config = init_config[:-1]
    Xerr_integ = np.zeros(6)
    logging.info('Generating the trajectory with PI feedback control')
    for i in range(0, N-1):
        # current_config = config_list[i-1,:-1]
        # print('current_config:', current_config)
        phi = current_config[0]
        x = current_config[1]
        y = current_config[2]
        Tsb = np.array([[np.cos(phi), -np.sin(phi), 0,      x],
                        [np.sin(phi),  np.cos(phi), 0,      y],
                        [          0,            0, 1, 0.0963],
                        [          0,            0, 0,      1]])
        current_jtangle = current_config[3: 8]
        
        T0e = mr.FKinBody(M0, Blist, current_jtangle)
        
        #Find X
        X = Tsb@Tb0@T0e
        
        # X = mr.FKinBody(Tse, Blist, current_jtangle)
        Xd = np.array([[traj_list[i][0], traj_list[i][1], traj_list[i][2], traj_list[i][9]],
                       [traj_list[i][3], traj_list[i][4], traj_list[i][5], traj_list[i][10]],
                       [traj_list[i][6], traj_list[i][7], traj_list[i][8], traj_list[i][11]],
                       [              0,               0,               0,               1]])
        Xd_next = np.array([[traj_list[i+1][0], traj_list[i+1][1], traj_list[i+1][2], traj_list[i+1][9]],
                            [traj_list[i+1][3], traj_list[i+1][4], traj_list[i+1][5], traj_list[i+1][10]],
                            [traj_list[i+1][6], traj_list[i+1][7], traj_list[i+1][8], traj_list[i+1][11]],
                            [                0,                 0,                 0,                 1]])
        V, control, Xerr, Xerr_integ = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt,current_config, Xerr_integ)

        current_config = NextState(current_config, control, dt, max_jtspeed, max_wheelspeed)
        config_list[i+1] = np.hstack((current_config, traj_list[i][12]))

        Xerr_plot[i] = Xerr
    #Generates the BEST csv one
    logging.info('Generating new task case animation csv file')
    np.savetxt("Results/NewTask/NewTask.csv", config_list, delimiter = ",")
    logging.info('Writing error plot data')
    np.savetxt("Results/NewTask/X_error.csv", Xerr_plot, delimiter = ",")

    x_lin = np.linspace(1, 20, N)
    plt.figure()
    plt.plot(x_lin, Xerr_plot[:, 0], label='Xerr 0')
    plt.plot(x_lin, Xerr_plot[:, 1], label='Xerr 1')
    plt.plot(x_lin, Xerr_plot[:, 2], label='Xerr 2')
    plt.plot(x_lin, Xerr_plot[:, 3], label='Xerr 3')
    plt.plot(x_lin, Xerr_plot[:, 4], label='Xerr 4')
    plt.plot(x_lin, Xerr_plot[:, 5], label='Xerr 5')
    plt.title(f'X error with time with Kp= {Kp_gain} and Ki= {Ki_gain}')
    plt.xlabel('Time(s)')
    plt.ylabel('Error')
    plt.legend(loc='best')
    plt.savefig(f'Results/NewTask/X_error New Task Kp= {Kp_gain} and Ki= {Ki_gain}.png')
    plt.show()

    fig, ax = plt.subplots(2, 3, figsize=(20,20))
    ax[0,0].plot(Xerr_plot[:, 0])
    ax[0,1].plot(Xerr_plot[:, 1])
    ax[0,2].plot(Xerr_plot[:, 2])
    ax[1,0].plot(Xerr_plot[:, 3])
    ax[1,1].plot(Xerr_plot[:, 4])
    ax[1,2].plot(Xerr_plot[:, 5])

    ax[0,0].set_title('Roll Error')
    ax[0,1].set_title('Pitch Error')
    ax[0,2].set_title('Yaw Error')


    ax[1,0].set_title('X Error')
    ax[1,1].set_title('Y Error')
    ax[1,2].set_title('Z Error')

    ax[0,0].set(xlabel='Time (s^-2)',ylabel='Error (rad)')
    ax[0,1].set(xlabel='Time (s^-2)',ylabel='Error (rad)')
    ax[0,2].set(xlabel='Time (s^-2)',ylabel='Error (rad)')
    ax[1,0].set(xlabel='Time (s^-2)',ylabel='Error (m)')
    ax[1,1].set(xlabel='Time (s^-2)',ylabel='Error (m)')
    ax[1,2].set(xlabel='Time (s^-2)',ylabel='Error (m)')

    fig.savefig(f'Results/NewTask/X_error New Task display Kp= {Kp_gain} and Ki= {Ki_gain}.png')
    plt.show()
    logging.info('Done')

if __name__ == "__main__":
    main()