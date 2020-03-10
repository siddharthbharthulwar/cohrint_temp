import numpy as np
import matplotlib.pyplot as plt


def lawnmowerpath(point1, point2, numOfTurns):


    #ascending vertically (horizontally simple)
    path = []
    x0 = point1[0]
    xf = point2[0]

    y0 = point1[1]
    yf = point2[1]

    scal = ((yf - y0) / numOfTurns)

    print(scal)

    for i in range(numOfTurns + 1):

        if (i % 2 == 0):

            path.append([x0, i * scal  + y0])
            path.append([xf, i * scal  + y0])

        else:

            path.append([xf, i * scal  + y0])
            path.append([x0, i * scal  + y0])

    return path


def plotpaths(paths):

    pathx = []
    pathy = []

    for i in range(len(paths)):

        pathx.append(paths[i][0])
        pathy.append(paths[i][1])
    
    plt.plot(pathx, pathy)
    plt.show()

paths = (lawnmowerpath((0, 0), (100, 100), 5))


'''
paths = np.array(paths)

plt.plot(paths[0, :])
plt.plot(paths[1, :])
plt.show()
'''

plotpaths(paths)
