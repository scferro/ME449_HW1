import numpy as np
import modern_robotics as mr

R13 = np.array([[-0.7071, 0, -0.7071], [0, 1, 0], [0.7071, 0, -0.7071]])
Rs2 = np.array([[-0.6964, 0.1736, 0.6964], [-0.1228, -0.9848, 0.1228], [0.7071, 0, 0.7071]])
R25 = np.array([[-0.7566, -0.1198, -0.6428], [-0.1564, 0.9877, 0], [0.6348, 0.1005, -0.7661]])
R12 = np.array([[0.7071, 0, -0.7071], [0, 1, 0], [0.7071, 0, 0.7071]])                                  # R12 (2)
R34 = np.array([[0.6428, 0, -0.7660], [0, 1, 0], [0.7660, 0, 0.6428]])                                  # R34 (4)
Rs6 = np.array([[0.9418, 0.3249, -0.0859], [0.3249, -0.9456, -0.0151], [-0.0861, -0.0136, -0.9962]])
R6b = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])

# Calculate the rotation matrixes between the links
R21 = R12.transpose()
R23 = np.matmul(R21, R13)        # R23 (3)
Rs1 = np.matmul(Rs2, R21)        # Rs1 (1)
R32 = R23.transpose()
R35 = np.matmul(R32, R25)   
R43 = R34.transpose()
R45 = np.matmul(R43, R35)        # R45 (5)
R2s = Rs2.transpose()
R26 = np.matmul(R2s, Rs6)   
R36 = np.matmul(R32, R26)   
R46 = np.matmul(R43, R36)  
R54 = R45.transpose()
R56 = np.matmul(R54, R46)        # R56 (6)
Rsb = np.matmul(Rs6, R6b)        # Rsb (7)

# Calculate Rs6 to validate calculations against the given Rs6
Rs2test = np.matmul(Rs1, R12)
Rs3test = np.matmul(Rs2test, R23)
Rs4test = np.matmul(Rs3test, R34)
Rs5test = np.matmul(Rs4test, R45)
Rs6test = np.matmul(Rs5test, R56)

# Calculate the exponential coordinates in skew-symmetric matrix form
skews1 = mr.MatrixLog3(Rs1)
skew12 = mr.MatrixLog3(R12)
skew23 = mr.MatrixLog3(R23)
skew34 = mr.MatrixLog3(R34)
skew45 = mr.MatrixLog3(R45)
skew56 = mr.MatrixLog3(R56)

# Extract the angle values from the exponential coordinates 
thetaMatrix = np.array([[skews1[2][1], skews1[0][2], skews1[1][0]]])
thetaMatrix = np.vstack([thetaMatrix, [skew12[2][1], skew12[0][2], skew12[1][0]]])
thetaMatrix = np.vstack([thetaMatrix, [skew23[2][1], skew23[0][2], skew23[1][0]]])
thetaMatrix = np.vstack([thetaMatrix, [skew34[2][1], skew34[0][2], skew34[1][0]]])
thetaMatrix = np.vstack([thetaMatrix, [skew45[2][1], skew45[0][2], skew45[1][0]]])
thetaMatrix = np.vstack([thetaMatrix, [skew56[2][1], skew56[0][2], skew56[1][0]]])

thetaVector = []
for row in thetaMatrix:
    sum = 0
    for item in row:
        sum += item
    thetaVector.append(sum)

print('thetaMatrix')
print(thetaMatrix)
print()
print('thetaVector')
print(thetaVector)
print()
print('Rsb')
print(Rsb)
print()
print('Rs6test')
print(Rs6test)