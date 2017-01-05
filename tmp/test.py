from pykalman.sqrt import BiermanKalmanFilter as KF2
import math
import numpy as np

T = 0.005;

PHI = np.array([[1, 0, T, 0], [0, 1, 0, T], [0, 0, 1, 0], [0, 0, 0, 1]])
H = np.array([[1, 0, 0, 0],[0, 1, 0, 0]])
R = np.array([[1.5083, 0.4], [0.4, 0.4965]])
Q = np.array([[pow(T,4)/4, 0, pow(T,3)/2, 0], [0, pow(T,4)/4, 0, pow(T,3)/2],[pow(T,3)/2, 0, pow(T,2), 0],[0, pow(T,3)/2, 0, pow(T,2)]])
x = np.array([[0, 0, 0, 0]])
P = np.array([[20, 0, 0, 0], [0, 20, 0, 0], [0, 0, 20, 0], [0, 0, 0, 20]])

measurements = np.asarray([10,5])


kf = KF2(PHI, H, Q, R)

#np.dot(PHI, np.array([0,0]))
print "############################"
print Q
print "############################"

(filter_state_means, filtered_state_covariance) = kf.filter_update(
            np.array([0, 0, 0, 0]),
            P,
            measurements,
            np.array([0,0,0,0])
        )

print filter_state_means
print "-------------------------------------"
print filtered_state_covariance
