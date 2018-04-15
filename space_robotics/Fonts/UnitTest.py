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
plane2 = [rotate([1.0,0,0], [np.pi/4, np.pi/4, np.pi/4]), rotate([0,1.0,0], [np.pi/4, np.pi/4, np.pi/4])]
plane3 = [rotate([1.0,0,0], [np.pi/2, np.pi/2, 0]), rotate([0,1.0,0], [np.pi/2, np.pi/2, 0])]

#render some sample text and stich the sequences together
pos = RenderText("ABC DEF", plane2, [0, 0, -2])
pos = np.r_[pos, RenderText("ABC DEF", plane1)]
pos = np.r_[pos, RenderText("CA BF ED", plane3, size = -0.3)]

#generate and show trajectory hints for one sequence
movements = []
movements.append(SequentializeText("ABC DEF", plane2, [0, 0, -2]))



#plot the output
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(pos[:,0],pos[:,1],pos[:,2])

for word in movements:
    for letter in word:
        for l in letter:
            ax.scatter(l[0][0], l[0][1], l[0][2])
            ax.scatter(l[1][0], l[1][1], l[1][2])
            if l[2] != [0,0,0]:
                ax.scatter(l[3][0],l[3][1],l[3][2], color = 'orange')



#rectify the axes
#workaround to force equal AS in 3D figures
X =  pos[:,0]
Y =  pos[:,1]
Z =  pos[:,2]
max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max()
Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(X.max()+X.min())
Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(Y.max()+Y.min())
Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(Z.max()+Z.min())

for xb, yb, zb in zip(Xb, Yb, Zb):
   ax.plot([xb], [yb], [zb], 'w')


plt.show()
