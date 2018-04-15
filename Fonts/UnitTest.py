import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from FontRender import RenderText, SequentializeText

#a simple rotation matrix function for easy plane definition
def rotate(vec, angles= [0,0,0], inv = False):
    phi, theta, psi = angles
    
    c= np.cos
    s= np.sin
    
    x= phi
    y= theta
    z= psi
    
    R = np.matrix([[c(y)*c(z), c(y)*s(z)  , -s(y)   ],
                   [s(x)*s(y)*c(z) - c(x)*s(z) , s(x)*s(y)*s(z) + c(x)*c(z) , s(x)*c(y)  ],
                   [c(x)*s(y)*c(z)+s(x)*s(z) , c(x)*s(y)*s(z)-s(x)*c(z)  , c(x)*c(y)  ]])
    if inv: R = R.T
    return np.array(vec*R)[0]

#create a few planes
#[x-vector, y-vector]

plane1 = [[-1,0,0], [0,-1,0]]
plane1 = [[1,0,0], [0,0,1]]
plane2 = [rotate([1.0,0,0], [np.pi/4, np.pi/4, np.pi/4]), rotate([0,1.0,0], [np.pi/4, np.pi/4, np.pi/4])]
plane3 = [rotate([1.0,0,0], [np.pi/2, np.pi/2, 0]), rotate([0,1.0,0], [np.pi/2, np.pi/2, 0])]

plane1 = plane2
#render some sample text and stich the sequences together
#pos = RenderText("ABC DEF", plane2, [0, 0, -2])
#pos = np.r_[pos, RenderText("ABCDEF", plane1)]

pos = RenderText("ABCDEFG", plane1)
pos2 = RenderText("HIJKLMN", plane1, origin = plane1[1]*-1.1)
pos3 = RenderText("OPRSTU", plane1, origin = plane1[1]*-2.2)
pos4 = RenderText("VWXYZ", plane1, origin = plane1[1]*-3.3)

hello = RenderText("HELLO", origin = [5,5,0], size = 2)
#pos = np.r_[pos, RenderText("CA BF ED", plane3, size = -0.3)]

#generate and show trajectory hints for one sequence
movements = []
movements.append(SequentializeText("HELLO", origin = [5,5,0], size = 2))



#plot the output
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(pos[:,0],pos[:,1],pos[:,2])
ax.plot(pos2[:,0],pos2[:,1],pos2[:,2])
ax.plot(pos3[:,0],pos3[:,1],pos3[:,2])
ax.plot(pos4[:,0],pos4[:,1],pos4[:,2])

ax.plot(hello[:,0],hello[:,1],hello[:,2])
# plt.plot(pos[:,0], pos[:,2])
# plt.axes().set_aspect('equal', 'datalim')
# plt.grid()


for word in movements:
    for letter in word:
        for l in letter:
            ax.scatter(l[0][0], l[0][1], l[0][2])
            ax.scatter(l[1][0], l[1][1], l[1][2])
            if l[2] != [0,0,0]:
                ax.scatter(l[3][0],l[3][1],l[3][2], color = 'orange')



# #rectify the axes
# #workaround to force equal AS in 3D figures
X =  np.r_[pos[:,0],pos4[:,0],hello[:,0]]
Y =  np.r_[pos[:,1],pos4[:,1],hello[:,1]]
Z =  np.r_[pos[:,2],pos4[:,2],hello[:,2]]
max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max()
Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(X.max()+X.min())
Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(Y.max()+Y.min())
Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(Z.max()+Z.min())

for xb, yb, zb in zip(Xb, Yb, Zb):
   ax.plot([xb], [yb], [zb], 'w')


plt.show()
