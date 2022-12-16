import numpy as np
import modern_robotics as mr

def NextState(cconfig, control, dt, maxspeed_jt, maxspeed_wheel):
    """
    A 12-vector representing the current configuration of the robot (3 variables for the chassis
     configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles).
    A 9-vector of controls indicating the wheel speeds u (4 variables) and the arm joint speeds 
    \dot{\theta} (5 variables).
    A timestep Δt.
    A positive real value indicating the maximum angular speed of the arm joints and the wheels. 
    For example, if this value is 12.3, the angular speed of the wheels and arm joints is limited 
    to the range [-12.3 radians/s, 12.3 radians/s]. Any speed in the 9-vector of controls that is 
    outside this range will be set to the nearest boundary of the range. If you don't want speed 
    limits, just use a very large number. If you prefer, your function can accept separate speed 
    limits for the wheels and arm joints. 
    The forward-backward distance between the wheels is 2l = 0.47 meters and the side-to-side 
    distance between wheels is 2w = 0.3 meters. The radius of each wheel is r = 0.0475 meters. 
    The forward driving and "free sliding" direction γ of each wheel is indicated.
    """
    jt_angles = cconfig[3:8]
    w_angles = cconfig[8:]
    jt_speed = control[4:]
    w_speed = control[:4]
    r = 0.0475
    l = 0.47/2
    w = 0.3/2
    qk = cconfig[:3]
    new_jtangles = jt_angles + jt_speed * dt
    new_wangles = w_angles + w_speed * dt
    for i in range(len(new_jtangles)):
        if new_jtangles[i] < -maxspeed_jt:
            new_jtangles[i] = -maxspeed_jt
        elif new_jtangles[i] > maxspeed_jt:
            new_jtangles[i] = maxspeed_jt
    for j in range(len(new_wangles)):
        if new_wangles[j] < -maxspeed_wheel:
            new_wangles[j] = -maxspeed_wheel
        elif new_wangles[j] > maxspeed_wheel:
            new_wangles[j] = maxspeed_wheel

    Vb = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                                    [1, 1, 1, 1], [-1, 1, -1, 1]]) @ w_speed
    w_bz = Vb[0]
    v_bx = Vb[1]
    v_by = Vb[2]
    dqb = np.array([0., 0., 0.])
    if w_bz == 0:
        dqb[1] = v_bx
        dqb[2] = v_by
    else:
        dqb[0] = w_bz
        dqb[1] = (v_bx * np.sin(w_bz) + v_by*(np.cos(w_bz)-1))/w_bz
        dqb[2] = (v_by*np.sin(w_bz) + v_bx*(1 - np.cos(w_bz)))/w_bz
    phik = cconfig[0]
    dq = np.array([[1, 0, 0],
                [0, np.cos(phik), -np.sin(phik)], 
                [0, np.sin(phik), np.cos(phik)]]) @ dqb
    qnext = qk + dq * dt
    result = np.concatenate([qnext, new_jtangles, new_wangles], axis=None)

    return result

def main():
    current_config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    speed = np.array([-10, 10, 10, -10, 0, 0, 0, 0, 0])
    result = np.zeros([100, 13])
    for i in range(100):
        result[i, :-1] = NextState(current_config, speed, 0.01, 5, 12.3)
        #Adding gripper state
        result[i, 12] = 0
        current_config = result[i, :-1]
    np.savetxt("Ghosh_Ritika_milestone1.csv", result, delimiter = ",")

if __name__ == "__main__":
    main()

