import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import utm



# Measurements Data processing
path_to_acc = './sensors_data/accelerometer.csv'
path_to_gps = './sensors_data/gps.csv'
data_acc = pd.read_csv(path_to_acc)
data_gps = pd.read_csv(path_to_gps)
acc = data_acc[['time', 'x','y']]
gps = data_gps[['time', 'lat','lng']]
gps.head()

#convert lattitude and lattitude into meters 
gps['x'], gps['y'], zone, u = utm.from_latlon(gps['lat'].values, gps['lng'].values)
gps.head()
t_acc = acc['time'].values
x_acc = acc['x'].values
y_acc = acc['y'].values
print(x_acc[0])
print(y_acc[0])

t_gps = gps['time']. values
x_gps = gps['x'].values
y_gps = gps['y'].values

plt.scatter(x_gps,y_gps)

plt.scatter(t_gps, x_gps, c='red', s=1)
plt.xlabel('time')
plt.ylabel('x coordinate')
plt.show()

plt.scatter(t_gps, y_gps, c='blue', s=1)
plt.xlabel('time')
plt.ylabel('x coordinate')
plt.show()

plt.scatter(t_acc, x_acc, c='red', s=1)
plt.xlabel('time')
plt.ylabel('x acceleration')
plt.show()

plt.scatter(t_acc, y_acc, c='blue', s=1)
plt.xlabel('time')
plt.ylabel('y acceleration')
plt.show()



#Interpolation
new_gps_x = np.interp(t_acc, t_gps, x_gps)
new_gps_y = np.interp(t_acc, t_gps, y_gps)
plt.scatter(t_acc, new_gps_x, c='red', s=1)
plt.xlabel('time')
plt.ylabel('new x gps after interpolation')
plt.show()
plt.scatter(t_acc, new_gps_y, c='blue', s=1)
plt.xlabel('time')
plt.ylabel('new y gps after interpolation')
plt.show()
plt.scatter(new_gps_x, new_gps_y, c='green', s=1)
plt.xlabel('time')
plt.ylabel('new y gps after interpolation')
plt.show()

#Extracting Accelerometer Distributions
acc_mean_x = x_acc.mean()
acc_std_x = x_acc.std()
print("Mean of acceletation x",acc_mean_x, "and std", acc_std_x )

acc_mean_y = y_acc.mean()
acc_std_y = y_acc.std()
print("Mean of acceletation y",acc_mean_y, "and std", acc_std_y )


#GUESS Correction
# Error in gps
gps_std_x = 10
gps_std_y = 10

#Measurement Covariation matrix
sigma_x = gps_std_x
sigma_y = gps_std_y
R = np.array([[sigma_x**2, 0],[0, sigma_y**2]], dtype = float)

#Process Covariation Matrix
def ProcessCovMatrix(dt,std_x, std_y):
    s_acc_x = std_x
    s_acc_y = std_y
    s_x = s_acc_x*dt**2/2
    s_dx = s_acc_x*dt
    s_y = s_acc_y*dt**2/2
    s_dy = s_acc_y*dt
    Q = np.array([[s_x**2, 0, s_x*s_dx, 0],
                  [0, s_y**2, 0, s_y*s_dy],
                  [s_dx*s_x, 0, s_dx*s_dx, 0], 
                  [0, s_dy*s_y, 0, s_dy*s_dy] ])
    return Q
#new GPS c

n = new_gps_x.shape[0]
print(n)
x_0 = new_gps_x[0]
y_0 = new_gps_y[0]
dx_0 = 0
dy_0 = 0
X_0 = np.reshape((np.array([x_0, y_0, dx_0, dy_0], dtype = float)), (-1,1))
print(X_0)
# State vector 
X = np.zeros((4,n), dtype = float)
X_p = np.zeros((4,n), dtype = float)
print(X_0.shape)
#Covariance Matrix
P = np.zeros((4,4, n), dtype = float)
P_p = np.zeros((4,4, n), dtype = float)

#Matrices configuration
def dynamicsMatrixA(dt):
    A = np.array([[1, 0,  dt, 0], 
                  [0, 1,  0, dt], 
                  [0, 0,  1, 0], 
                  [0, 0, 0, 1]], dtype = float)
    return A
def controlMatrixB(dt):
    B = np.array([[1/2*dt**2, 0], 
                  [0, 1/2*dt**2], 
                  [dt, 0], 
                  [0, dt]], dtype = float)
    return B
#Control matrix u 
uc = np.array(([x_acc, y_acc]), dtype = float)
print(uc.shape)
print(uc[:, 1])

#Measurement matrices
C = np.array([[1, 0], [0, 1]], dtype = float)
H = np.array([[1., 0., 0., 0.],  [0., 1., 0., 0.]], dtype = float)
print("gps ", new_gps_x.shape)
Ym = np.array(([new_gps_x, new_gps_y]), dtype = float)
print("Ym", Ym.shape)
Y = np.ones((2, n), dtype = float)
Y[:, 0] = Ym[:, 0]
H_new = np.linalg.pinv(H)
print(H_new.shape)

#Distributions
#Measurement Noise
zm = np.array([[np.random.normal(0, 0)], [np.random.normal(0, 0)]], dtype = float)

def modelNoise(dt, std_x, std_y):
    s_acc_x = std_x
    s_acc_y = std_y
#     s_x = (s_acc_x*dt**2)/2
#     s_dx = s_acc_x*dt
#     s_y = (s_acc_y*dt**2)/2
#     s_dy = s_acc_y*dt
    s_x = 0.01
    s_dx = 0.02
    s_y = 0.01
    s_dy = 0.02
    r = np.array([[np.random.normal(0, s_x)], [np.random.normal(0, s_y)], [np.random.normal(0, s_dx)], [np.random.normal(0, s_dy)]], dtype = float)
    w = np.reshape(r,(4,))
    return w
#TIME CON

time = t_acc/1000
X[:,0] = X_0[:,0]
print(X[:, 0])
P_0 = np.zeros((4,4), dtype = float)
P[:,:,0] = P_0[:,:]

#LOOPING
for k in range(1, n):
    dt = time[k] - time[k-1]
    A = dynamicsMatrixA(dt)
    B = controlMatrixB(dt)
    Q = ProcessCovMatrix(dt, acc_std_x, acc_std_y )
    w = modelNoise(dt, acc_std_x, acc_std_y)
    #print(np.matmul(A, X[:, k-1]).shape)
    
    ## PREDICTION STAGE
    # X predicted
    X_p[:, k] = np.dot(A, X[:, k-1]) + np.dot(B, uc[:,k-1]) + w
    #State covariance matrix
    P_p[:, :, k] = np.dot(A, np.dot(P[:, :, k-1], A.transpose())) + Q
    
    ## UPDATE STATE
    # Kalman gain
    K = np.dot(np.dot(P_p[:, :, k], H.transpose()), np.linalg.pinv(np.dot(H, np.matmul(P_p[:, :, k], H.transpose())) + R))
    # Measurements 
    #print("", zm.shape)
    Y[:, k] = np.matmul(C, Ym[:, k]) + zm[:,0]
    # Estimate the update 
    update = np.dot(K, Y[:, k] - np.dot(H, X_p[:, k]))
    #Updated X
    X[:, k] = X_p[:, k] + update
    # Updated P
    I = np.eye(4, k=0)
    P[:, :, k] = np.dot((I - np.dot(K, H)), P_p[:, :, k])

print(X.shape)
x_opt = X[0]
y_opt = X[1]
print(X[0])

plt.scatter(x_opt, y_opt, c='red', s=1)
plt.xlabel('time')
plt.ylabel('new x gps after interpolation')
plt.show()
