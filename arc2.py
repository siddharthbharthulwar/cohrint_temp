from math import pi, hypot, sin, cos, atan2, degrees

def norm_angle(a):
    # Normalize the angle to be between -pi and pi
    return (a+pi)%(2*pi) - pi

# Given values
# named just like in http://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePath.htm
x_0, y_0 = [400,500] # initial position of robot
theta_0 = -pi/2      # initial orientation of robot
s = 10               # speed of robot
x_1, y_1 = [0,300] # goal position of robot
t_0 = 0              # starting time

# To be computed:
r_G = hypot(x_1 - x_0, y_1 - y_0)        # relative polar coordinates of the goal
phi_G = atan2(y_1 - y_0, x_1 - x_0)
phi = 2*norm_angle(phi_G - theta_0)      # angle and 
r_C = r_G/(2*sin(phi_G - theta_0))       # radius (sometimes negative) of the arc
L = r_C*phi                              # length of the arc
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

def position(t):
    x = x_C + r_C*sin(omega*(t - t_0) + theta_0)
    y = y_C - r_C*cos(omega*(t - t_0) + theta_0)
    return x, y

def orientation(t):
    return omega*(t - t_0) + theta_0

#--------------------------------------------
# Just used for debugging
#--------------------------------------------
import turtle

screen = turtle.Screen()
screen.setup(600, 600)
screen.setworldcoordinates(0, 0, 600, 600)

turtle.hideturtle()
turtle.shape("turtle")
turtle.penup()
turtle.goto(x_1, y_1)
turtle.setheading(degrees(orientation(t_1)))
turtle.stamp()
turtle.goto(x_0, y_0)
turtle.color("red")
turtle.showturtle()
turtle.pendown()
for t in range(t_0, int(t_1)+1):
    turtle.goto(*position(t))
    turtle.setheading(degrees(orientation(t)))