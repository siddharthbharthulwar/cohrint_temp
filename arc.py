from math import pi, hypot, sin, cos, atan2, degrees, radians

def norm_angle(a):
    # Normalize the angle to be between -pi and pi
    return (a+pi)%(2*pi) - pi

def computeArcs(pos1, pos2, theta_0, s, t_0):
    theta_0 = radians(theta_0)
    x_0 = pos1[0]
    y_0 = pos1[1]
    x_1 = pos2[0]
    y_1 = pos2[1]
    r_G = hypot(x_1 - x_0, y_1 - y_0)  
    phi_G = atan2(y_1 - y_0, x_1 - x_0)
    phi = 2*norm_angle(phi_G - theta_0) 
    r_C = r_G/(2*sin(phi_G - theta_0))  
    L = r_C*phi  
    if phi > pi:
        phi -= 2*pi
        L = -r_C*phi
    elif phi < -pi:
        phi += 2*pi
        L = -r_C*phi
    t_1 = L/s + t_0                        # time at which the robot finishes the arc
    omega = phi/(t_1 - t_0)                # angular velocity           
    x_C = x_0 - r_C*sin(theta_0)           # center of rotation
    y_C = y_0 + r_C*cos(theta_0)
    ret = {
        "x_C": x_C,
        "y_C": y_C,
        "r_C": r_C,
        "t_0": t_0,
        "t_1": t_1,
        "theta_0": theta_0,
        "omega": omega

    }
    return ret
def position(t, ret):
    x_C = ret["x_C"]
    y_C = ret["y_C"]
    r_C = ret["r_C"]
    t_0 = ret["t_0"]
    theta_0 = ret["theta_0"]
    omega = ret["omega"]
    x = x_C + r_C*sin(omega*(t - t_0) + theta_0)
    y = y_C - r_C*cos(omega*(t - t_0) + theta_0)
    return x, y
def orientation(t, ret):
    x_C = ret["x_C"]
    y_C = ret["y_C"]
    r_C = ret["r_C"]
    t_0 = ret["t_0"]
    theta_0 = ret["theta_0"]
    omega = ret["omega"]
    return omega * (t - t_0) + theta_0

def totalcomputation(pos0, pos1, t_0, theta_0, s):
    ret = computeArcs(pos0, pos1, theta_0, s, t_0)
    positions = []
    headings = []
    for t in range(t_0, int(ret["t_1"]) + 1):
        positions.append(position(t, ret))
        headings.append((orientation(t, ret)))
    return positions, headings

print(totalcomputation((0, 0), (500, 500), 0, 15, 10)[1])

