import numpy as np
from Font import Font

def renderCircle(start, end, center, dir):
    "approximate a circle"
    a0 = np.arctan2(start[1] - center[1], start[0]- center[0] )
    a1 = np.arctan2(end[1] - center[1], end[0] - center[0])
    r = ((center[1] - start[1])**2 + (center[0] - start[0])**2)**0.5

    #some quick and dirty angle rules just to draw
    if dir > 0:
        a1 -= np.pi * 2
    elif dir < 1 and a1 < 1 and not a0 < 0:
        a1 += np.pi *2
  
    steps = np.linspace(a0, a1)
    
    return np.c_[np.cos(steps)*r + center[0], np.sin(steps)*r + center[1], np.zeros(len(steps))]
    
def RenderChar(c, plane = [[1,0,0], [0,1,0]], origin = [0,0,0], size = 1.0):
    "render a character onto a plane"
    #fetch the splines
    sequence = Font[c]

    #render as 2D segments
    splines = np.array([[sequence[0][0][0],sequence[0][0][1], 0]])

    for s in sequence:
        if s[2][2] == 0:
            splines = np.r_[splines, np.array([[s[1][0],s[1][1], 0]])]
        elif s[2][2] in [-1, 1]:
            splines = np.r_[splines, renderCircle(s[0], s[1], s[2][:2], s[2][2])]

    #project to 3D plane
    vertex = np.zeros(splines.shape)
    for i in xrange(len(vertex)):
        vertex[i] = splines[i][0] * plane[0] + splines[i][1] * plane[1]
    
    #apply origin offset and return the sequence
    return vertex*size + np.array(origin)

def RenderText(sr, plane = [[1,0,0], [0,1,0]], origin = [0,0,0], size = 1.0):
    "renders a string of text on a plane"
    #normalize the plane vectors
    plane = np.array(plane).astype(float)
    plane[0] /= np.sqrt(np.sum(plane[0]**2))
    plane[1] /= np.sqrt(np.sum(plane[1]**2)) 
    plane = plane / np.sqrt(np.sum(plane**2)) * 2**0.5

    first = True
    pos = []
    origint = np.array(origin)

    for c in sr:
        if first:
            #no idea why this is a problem, but plane array is not loaded into this scope due to try/except block
            origint = origint +0 * plane[0]
        if c == ' ':
            origint = origint+ 1.1*plane[0] * size
            continue
        try:
            #try to draw a character, font doesnt cover all 256 charactes so just skip whatever is missing. Just like IE!
            if first:
                pos = RenderChar(c, plane, origint, size)
                first = False
            else:
                pos = np.r_[pos, RenderChar(c, plane, origint, size)]
        except:
            continue

        origint += 1.1*plane[0] * size
        #print origin
    return pos


### expand to movement points for trajectory
# format: [ [start xyz, end xyz, curvature flag, center xyz], ... ]
# curvature flags are: [0,0,0] for straight segment, [-1,-1,-1] for counter clockwise, [1,1,1] for clockwise


def SequentializeCharacter(c, plane = [[1,0,0], [0,1,0]], origin = [0,0,0], size = 1.0):
    "Returns a plane projected set of reference points for a single character"

    #again, fetch the character data    
    sequence = Font[c]

    origin = np.array(origin)
    
    movements = []
    
    for s in sequence:

        tmp = []
        tmp.append(size*(plane[0]*s[0][0] + plane[1]*s[0][1]) + origin)
        tmp.append(size*(plane[0]*s[1][0] + plane[1]*s[1][1]) + origin)
        
        if s[2][2] == 0:
            tmp.append([0,0,0])
        else:
            tmp.append([s[2][2],s[2][2],s[2][2]])

        tmp.append(size*(plane[0]*s[2][0] + plane[1]*s[2][1]) + origin)
        movements.append(tmp)
    return movements

def SequentializeText(sr, plane = [[1,0,0], [0,1,0]], origin = [0,0,0], size = 1.0):
    "Returns a full sequence of plane projected reference points for trajectory"
    
    #normalize the plane vectors
    plane = np.array(plane).astype(float)
    plane[0] /= np.sqrt(np.sum(plane[0]**2))
    plane[1] /= np.sqrt(np.sum(plane[1]**2)) 
    plane = plane / np.sqrt(np.sum(plane**2)) * 2**0.5

    first = True
    origint = np.array(origin)

    movements = []

    for c in sr:
        if first:
            #no idea why this is a problem, but plane array is not loaded into this scope due to try/except block
            origint = origint +0 * plane[0]
            first = False
        if c == ' ':
            origint = origint+ 1.1*plane[0] * size
            continue
        try:
            #try to draw a character, font doesnt cover all 256 charactes so just skip whatever is missing. Just like IE!
            movements.append(SequentializeCharacter(c, plane, origint, size))
        except:
            continue

        origint += 1.1*plane[0] * size
        #print origin
    return movements