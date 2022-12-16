import numpy as np
from modern_robotics import * 


def FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt, config, Xerr_prev=np.zeros(6)):
    """
    Args:
    The current actual end-effector configuration X (also written Tse).
    The current end-effector reference configuration Xd (i.e., Tse,d).
    The end-effector reference configuration at the next timestep in the reference trajectory, Xd,next (i.e., Tse,d,next), at a time Δt later.
    The PI gain matrices Kp and Ki.
    The timestep Δt between reference trajectory configurations. 
    Output:
    The commanded end-effector twist {V} expressed in the end-effector frame {e}. 
    """
    Xerr_se3 = MatrixLog6(TransInv(X)@Xd)
    Xerr = se3ToVec(Xerr_se3)
    Vd_se3 = MatrixLog6(TransInv(Xd)@Xdnext)/dt
    Vd = se3ToVec(Vd_se3)
    V = Adjoint(TransInv(X)@Xd)@Vd + Kp@Xerr + Ki@(Xerr_prev + Xerr*dt)
    r = 0.0475
    l = 0.47/2
    w = 0.3/2
    F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                        [1, 1, 1, 1],
                        [-1, 1, -1, 1]])
    F6 = np.array([[0, 0, 0, 0], 
                    [0, 0, 0, 0],
                    F[0],
                    F[1], 
                    F[2], 
                    [0, 0, 0, 0]])

    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,      1]])
    Mo = np.array([[1, 0, 0,  0.033],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0,      1]])
    Blist = np.array([[0,  0, 1,       0, 0.033, 0],
                      [0, -1, 0, -0.5076,     0, 0],
                      [0, -1, 0, -0.3526,     0, 0],
                      [0, -1, 0, -0.2176,     0, 0],
                      [0,  0, 1,       0,     0, 0]]).T
    thetalist = config[3:8]
    T0e = FKinBody(Mo, Blist, thetalist)
    Jb = Adjoint(TransInv(T0e)@TransInv(Tb0))@F6
    Jarm = JacobianBody(Blist, thetalist)
    Je = np.hstack([Jb, Jarm])
    speed = np.linalg.pinv(Je, 1e-3)@V
    return V, speed, Xerr, Xerr_prev + Xerr*dt

def main():
    X = np.array([[0.170, 0, 0.985, 0.387], [0, 1, 0, 0], [-0.985, 0, 0.170, 0.570], [0, 0, 0, 1]])
    Xd = np.array([[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
    Xd_next = np.array([[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]])
    Kp = np.identity(6)
    Ki = np.zeros([6, 6])
    Xerr_integ = np.zeros(6)
    config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
    V, speed, Xe, Xerr_integ = FeedbackControl(X, Xd, Xd_next, Kp, Ki, 0.01, config, Xerr_integ)

if __name__ == "__main__":
    main()
