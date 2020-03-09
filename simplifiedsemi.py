import numpy as np 

point0 = [0, 0]
point1 = [100, 100]


midpoint = [50, 50]

vector0 = (0 - 50, 0  - 50)
vector1 = (100 - 50, 100 - 50)


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def anglesbetween(angle, num):
    difference = angle / num
    angles = []
    for i in range(1, num + 1):
        angles.append(i * difference)
    return angles

angle = angle_between(vector0, vector1)
print(anglesbetween(angle, 10))