
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
from numpy import linalg as LA
import math

# Make this true to use the 3D renderer
useCPP_Renderer = False

if useCPP_Renderer:
    import os

    # Change this directory if necessary
    os.add_dll_directory(r"C:\msys64\ucrt64\bin")
    from ctypes import *

    libc2 = CDLL(r"Renderer3D\build\libOut")


    class RENDERDATA(Structure):
        _fields_ = [("TIME", c_double),
                    ("POS_X", c_double),
                    ("POS_Y", c_double),
                    ("POS_Z", c_double),
                    ("Q_W", c_double),
                    ("Q_X", c_double),
                    ("Q_Y", c_double),
                    ("Q_Z", c_double),
                    ("THRUST", c_double)]


    libc2.addData.argtypes = [RENDERDATA]

    libc2.mainRender.restype = c_int
    libc2.mainRender()


def quat_multi(a, b):
    """
    Returns a*b, where 'a' and 'b' are quaternions
    """
    return np.array([a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                     a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                     a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                     a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]])

def quat_get_euler_angles(q):
    """
    Returns the Tait-Bryan angle representation as (pitch, yaw, roll) from the
    provided quaternion 'q' in radians. When the rocket is pointing up (0,0,0,1), the angle returned is (0,0,0).
    """
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(2.0 * (w * y - z * x))
    yaw = math.atan2(2.0 * (w * z + y * x), 1.0 - 2.0 * (z * z + y * y))
    return np.array((roll, pitch, yaw))

def quat_inv(q):
    return np.array((q[0], -q[1], -q[2], -q[3]))

def quat_rotate_vec(q, v):
    """
    Does (q*v)*(q^-1), where 'q' is a quaternion and 'v' is a 3d vector
    Returns such result as a 3d vector (i,j,k), aka the rotation of the vector by 'q'
    """
    q_v = np.array((0.0, v[0], v[1], v[2]))
    return quat_multi(quat_multi(q, q_v), quat_inv(q))[1:4]

def quat_to_mat(q):
    """
    Turns the provided quaterion as a rotation matrix
    """
    q /= LA.norm(q)
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    xx = 1.0 - 2.0 * (qy * qy + qz * qz)
    xy = 2.0 * (qx * qy + qw * qz)
    xz = 2.0 * (qx * qz - qw * qy)

    yx = 2.0 * (qx * qy - qw * qz)
    yy = 1.0 - 2.0 * (qx * qx + qz * qz)
    yz = 2.0 * (qw * qx + qy * qz)

    zx = 2.0 * (qw * qy + qx * qz)
    zy = 2.0 * (qy * qz - qw * qx)
    zz = 1.0 - 2.0 * (qx * qx + qy * qy)

    return np.array([[xx,xy,xz],
                     [yx,yy,yz],
                     [zx,zy,zz]])

def quat_from_vec_angle(v, angle):
    return np.array([math.cos(angle / 2.0), math.sin(angle / 2.0) * v[0], math.sin(angle / 2.0) * v[1], math.sin(angle / 2.0) * v[2]])

F15EstesThrust = [
    [0.000, 10.00],
    [0.063, 10.127],
    [0.118, 10.407],
    [0.158, 10.359],
    [0.228, 13.68],
    [0.340, 20.82],
    [0.386, 26.75],
    [0.425, 25.38],
    [0.481, 22.19],
    [0.583, 17.93],
    [0.883, 16.11],
    [1.191, 14.59],
    [1.364, 15.35],
    [1.569, 15.65],
    [1.727, 14.74],
    [2.000, 14.28],
    [2.390, 13.68],
    [2.680, 13.08],
    [2.960, 13.07],
    [3.250, 13.05],
    [3.350, 13.00],
    [3.390, 7.30],
    [3.400, 0.00],
]

BURN_TIME = F15EstesThrust[-1][0]
SECOND_IGNITION = 4.38 # The time of which the second motor starts in seconds

wind_speed = 5.0 # in m/s

class Rocket:
    # The mass of the rocket without any motors
    rocket_mass = 1.2
    motor_mass_full =  0.094 # 94.0 g
    motor_mass_empty = 0.06 # 60.0 g

    # The total mass is init_mass + motor_mass_full*2

    # In meters (m)
    diameter = 0.076
    height = 0.68
    # The area of the projection of the rocket from different angles
    A_side = diameter * height
    A_top = (diameter/2) ** 2 * math.pi

    # Distance from the top of the rocket in meters
    # These motors will be extending a bit out of the rocket
    motor_1_length = height + 0.08
    motor_2_length = height + 0.02

    I_x = 1.0/12 * rocket_mass * (3*diameter**2 + height**2) * 0.5
    I_y = I_x
    I_z = 0.5 * rocket_mass * diameter**2

    # The second moment of inertia, is based on the rocket's frame and not on the world's frame
    inertia = np.array([[I_x, 0.0, 0.0],
                        [0.0, I_y, 0.0],
                        [0.0, 0.0, I_z]])

    center_of_mass_without_motors = height*3/5 # as a distance

    center_of_pressure = 0.1 # as a distance

    C_drag = 0.5

def update_rocket_mass(mass):
    Rocket.rocket_mass = mass

    Rocket.I_x = 1.0 / 12 * Rocket.rocket_mass * (3 * Rocket.diameter ** 2 + Rocket.height ** 2) * 0.5
    Rocket.I_y = Rocket.I_x
    Rocket.I_z = 0.5 * Rocket.rocket_mass * Rocket.diameter ** 2

    Rocket.inertia = np.array([[Rocket.I_x, 0.0, 0.0],
                               [0.0, Rocket.I_y, 0.0],
                               [0.0, 0.0, Rocket.I_z]])

def get_thrust(time) -> float:
    """
    Linear interpolation of thrust curve.
    Returns the current thrust at time from the graph 'F15EstesThrust' as a float
    """
    if time <= F15EstesThrust[0][0]:
        return F15EstesThrust[0][1] if time >= 0.0 else 0.0
    for i in range(len(F15EstesThrust) - 1):
        t0, f0 = F15EstesThrust[i]
        t1, f1 = F15EstesThrust[i + 1]
        if t0 <= time < t1:
            return f0 + (f1 - f0) * (time - t0) / (t1 - t0)
    return 0.0


def get_total_thrust(time):
    T = max(get_thrust(time), get_thrust(time - SECOND_IGNITION))
    return np.array([0.0,0.0,T])

def tvc_get_error(q):
    """
    Returns the error of the tvc as a 2d vec (x,y)
    """

    q_w = q[0]
    q_x = q[1]
    q_y = q[2]
    q_z = q[3]

    zx = 2.0 * (q_x * q_z + q_w * q_y)
    zy = 2.0 * (q_y * q_z - q_w * q_x)

    return np.array([zy, -zx])


total_error = np.array([0.0,0.0])

def tvc_pid(q, error, w):
    """
    Returns the target tvc values as (x,y)
    """
    global total_error

    K_p = 0.45
    K_i = 1.2
    K_d = 0.08

    quat_mat = quat_to_mat(q)
    r = quat_mat[2]
    tvc_x = quat_mat[0]
    tvc_y = quat_mat[1]

    d_error_x = w[1] * r[2] - w[2] * r[1]
    d_error_y = w[2] * r[0] - w[0] * r[2]

    derivative_x = d_error_y
    derivative_y = -d_error_x

    # The increase in total error is multiplied by 0.25 to account
    # for this code running four times per step in the simulation
    total_error += error * dt * 0.25

    tvc_total_error_max = math.radians(5.0)
    tvc_total_error_min = -tvc_total_error_max
    total_error[0] = max(min(total_error[0], tvc_total_error_max), tvc_total_error_min)
    total_error[1] = max(min(total_error[1], tvc_total_error_max), tvc_total_error_min)

    angle = -(K_p * error + K_d * np.array([derivative_x, derivative_y]) + K_i * total_error)*2
    tvc_max = math.radians(7.0)
    tvc_min = -math.radians(7.0)
    angle = (max(min(angle[0], tvc_max), tvc_min), max(min(angle[1], tvc_max), tvc_min))

    angle = (-angle[1], angle[0])

    TVC_X = angle[1] * r[2]
    TVC_Y = -angle[0] * r[2]
    TVC_Z = angle[0] * r[1] - angle[1] * r[0]
    TVC = np.array((TVC_X, TVC_Y, TVC_Z))

    TARGET_TVC_X = LA.vecdot(tvc_x, TVC)
    TARGET_TVC_Y = LA.vecdot(tvc_y, TVC)
    return np.array((TARGET_TVC_X, TARGET_TVC_Y))


def get_motor_mass(time, offset, release: bool):
    """
    This uses linear interpolation to find the mass, which may or may not be accurate, as
    the motor may change from time how fast mass is depleted
    release - tells if the motor will be released from the rocket
    Returns the motor's mass at the given time
    """
    if time-offset <= 0.0:
        return Rocket.motor_mass_full

    if time-offset >= BURN_TIME:
        if release and time-offset - 0.1 >= BURN_TIME:
            return 0.0
        else:
            return Rocket.motor_mass_empty

    return Rocket.motor_mass_full + (Rocket.motor_mass_empty - Rocket.motor_mass_full) * (time-offset)/BURN_TIME


def get_mass(time):
    mass = Rocket.rocket_mass
    mass += get_motor_mass(time, 0.0, True)
    mass += get_motor_mass(time, SECOND_IGNITION, False)
    return mass

def get_center_of_mass(time):
    """
    Note that this gives the pos from the reference to the rocket,
    and therefore is not rotated by the rocket's quaternion.
    :returns: The CoM in form of a 3d vec: <0, 0, distance>, where distance is from the top of the rocket
    """
    sum_pos = Rocket.center_of_mass_without_motors * Rocket.rocket_mass
    sum_pos += get_motor_mass(time, 0.0, True) * Rocket.motor_1_length
    sum_pos += get_motor_mass(time, SECOND_IGNITION, False) * Rocket.motor_2_length
    sum_mass = Rocket.rocket_mass + get_motor_mass(time, 0.0, True) + get_motor_mass(time, SECOND_IGNITION, False)
    return np.array([0.0, 0.0, sum_pos/sum_mass])

def get_wind(time):
    """ Gets wind as a 3d vec"""
    time *= 0.1
    time += math.radians(30.0)
    return np.array([math.cos(time)*wind_speed, -math.sin(time)*wind_speed, 0.0])



def get_drag_force(rel_vel, q):

    if LA.norm(rel_vel) < 0.0001:
        return np.array([0.0, 0.0, 0.0])

    c_d = Rocket.C_drag  # Constant for now
    p = 1.2  # mass density of air at sea pressure
    # Areas are in meters^2
    # The area of the projection of the rocket from different directions
    A_side = Rocket.A_side
    A_top = Rocket.A_top
    up_vec = quat_to_mat(q)[2]

    angle_of_attack = 0.0
    if LA.norm(rel_vel) > 0.01:
        angle_of_attack = math.acos(np.dot(-rel_vel, up_vec)/(LA.norm(rel_vel)))
    A = A_top * abs(math.cos(angle_of_attack)) + A_side * abs(math.sin(angle_of_attack))

    drag_force = 0.5 * p * rel_vel * LA.norm(rel_vel) * c_d * A

    total_vel_norm = rel_vel/LA.norm(rel_vel)
    lift_dir = up_vec - total_vel_norm * LA.vecdot(up_vec, total_vel_norm)
    lift_dir = lift_dir/LA.norm(lift_dir)
    c_lift = 3.0 * angle_of_attack * math.exp(-(angle_of_attack/math.radians(20.0))**4)
    lift_force = 0.5 * p * LA.norm(rel_vel) * LA.norm(rel_vel) * c_lift * A * lift_dir

    return drag_force + lift_force


def get_total_force(time, vel, q, tvc_angles):
    """ Based on F = F_T + F_D + F_g """
    rel_vel = get_wind(time) - vel

    tvc_quat = quat_multi(quat_from_vec_angle([1.0,0.0,0.0], tvc_angles[0]),
                          quat_from_vec_angle([0.0,1.0,0.0], tvc_angles[1]))

    thrust = quat_rotate_vec(q, quat_rotate_vec(tvc_quat, get_total_thrust(time)))
    return thrust + get_drag_force(rel_vel, q)


def calc_accel(time, vel, q, tvc_angles):
    gravity = np.array([0.0, 0.0, -9.8])
    return get_total_force(time, vel, q, tvc_angles) / get_mass(time) + gravity


def get_total_torque(time, vel, q, tvc_angles):
    """
    Torque is based on the world frame
    """
    CoM = get_center_of_mass(time)
    motor_1_pos = CoM - np.array([0.0, 0.0, Rocket.motor_1_length])
    motor_2_pos = CoM - np.array([0.0, 0.0, Rocket.motor_2_length])
    drag_pos = CoM - np.array([0.0, 0.0, Rocket.center_of_pressure])

    total_torque = np.array([0.0, 0.0, 0.0])

    tvc_quat = quat_multi(quat_from_vec_angle([1.0, 0.0, 0.0], tvc_angles[0]),
                          quat_from_vec_angle([0.0, 1.0, 0.0],tvc_angles[1]))
    thrust_vec = quat_rotate_vec(q, quat_rotate_vec(tvc_quat, get_total_thrust(time)))

    if 0.0 <= time < BURN_TIME:
        total_torque = LA.cross(quat_rotate_vec(q, motor_1_pos), thrust_vec)
    elif SECOND_IGNITION <= time < BURN_TIME+SECOND_IGNITION:
        total_torque = LA.cross(quat_rotate_vec(q, motor_2_pos), thrust_vec)
    rel_vel = get_wind(time) - vel
    total_torque += LA.cross(quat_rotate_vec(q, drag_pos), get_drag_force(rel_vel, q))
    return total_torque


def get_angular_accel(time, vel, w, q, tvc_angles):
    """
    Angular acceleration is based on the world frame
    """

    # Euler's rotation equation:
    # I*w_dot + w cross (I*w) = M
    # w_dot = I^-1 * (M - w cross (I*w))
    # M - net torque
    # I - inertia matrix
    # w_d - angular accel
    q_mat = quat_to_mat(q)
    I = q_mat @ Rocket.inertia @ np.transpose(q_mat)
    M = get_total_torque(time, vel, q, tvc_angles)
    return LA.inv(I) @ (M - LA.cross(w, (I@w)))



def runge_kutta(t, u, differential_f, delta_t):
    """
    Does a 4th order runge kutta for a first order diff equation
    differential_f - function that returns du/dt for a given t and u
    """
    k1 = differential_f(t, u)
    k2 = differential_f(t + delta_t / 2, u + (delta_t / 2) * k1)
    k3 = differential_f(t + delta_t / 2, u + (delta_t / 2) * k2)
    k4 = differential_f(t + delta_t, u + delta_t * k3)

    return u + (delta_t / 6) * (k1 + 2 * k2 + 2 * k3 + k4)



initial_time = 0.0 # The initial time of the simulation
final_time = 10.0 # The final time of the simulation
dt = 0.05

# Number of time steps
num_steps = int(round((final_time - initial_time) / dt))
# Arrays for storing the solution
t_values = np.linspace(initial_time, final_time, num_steps + 1)
u_values = np.zeros((num_steps + 1, 3+3+4+3))
# u[0:2]   is r
# u[3:5]   is v
# u[6:9]   is q
# u[10:12] is w
u_values[0][0:3]   = np.array([0.0, 0.0, 0.0])        # initial for pos
u_values[0][3:6]   = np.array([0.0, 0.0, 0.0])        # initial for vel
u_values[0][6:10]  = np.array([1.0, 0.05, 0.0, 0.0])  # initial for quaternion
u_values[0][6:10]  /= LA.norm(u_values[0][6:10])  # normalize quaternion
u_values[0][10:13] = np.array([0.0, 0.0, 0.0])        # initial for angular vel




def diff_u(t,u):
    """
    u is a matrix of the values [p (0), v (1), q (2), w (3)]
    This returns the rate of change of u, or u dot,
    as a matrix of [p_dot, v_dot, q_dot, w_dot] = [v, a, q_dot, w_dot]
    """

    v = u[3:6]
    q = u[6:10]
    w = u[10:13]

    tvc_angles = tvc_pid(q, tvc_get_error(q), w)

    result = np.zeros((3+3+4+3))
    result[0:3]   = v
    result[3:6]   = calc_accel(t, v, q, tvc_angles)
    result[6:10]  = 0.5*quat_multi(np.array([0.0, w[0], w[1], w[2]]), q)
    result[10:13] = get_angular_accel(t, v, w, q, tvc_angles)

    return result

def update_sim():

    # Reset the PID to its initial state
    global total_error
    total_error = np.array([0.0,0.0])

    # Numerical solution using RK4
    for i in range(num_steps):
        u_values[i + 1] = runge_kutta(t_values[i], u_values[i], diff_u, dt)
        u_values[i + 1][6:10] /= LA.norm(u_values[i + 1][6:10])

        # Prevents the rocket from having a -z, and acts like the ground
        if u_values[i + 1][2] < 0:
            u_values[i + 1][2] = 0.0
            u_values[i + 1][5] = max(u_values[i + 1][5], 0.0)

update_sim()


fig2d = plt.figure(figsize=(8, 5))
ax2d = fig2d.add_subplot()
plt.subplots_adjust(bottom=0.2,top=0.9)

value_list = []
value_list_3d = [[], [], []]

def update_graph2d():
    # Plot numerical and exact solutions
    global value_list, value_list_3d
    value_list = []
    value_list_3d = [[], [], []]

    ax2d.clear()

    for i in range(len(t_values)):
        angles = np.degrees(quat_get_euler_angles(u_values[i][6:10]))
        value_list.append((u_values[i][2], u_values[i][5],
                           angles[0], angles[1], angles[2]))
        value_list_3d[0].append(u_values[i][0])
        value_list_3d[1].append(u_values[i][1])
        value_list_3d[2].append(max(u_values[i][2], 0.0))

    ax2d.plot(t_values, value_list, label=["pos.z", "vel.z",
                                                 "roll", "pitch", "yaw"])

    ax2d.axvline(SECOND_IGNITION)

    ax2d.set_xlabel("time")
    ax2d.legend()
    ax2d.grid()
    ax2d.set_title("Rocket Sim")
    plt.show()


    if useCPP_Renderer:
        libc2.clearData()

        for i in range(u_values.shape[0]):
            libc2.addData(RENDERDATA(t_values[i],
                                     u_values[i][0],
                                     u_values[i][1],
                                     u_values[i][2],
                                     u_values[i][6],
                                     u_values[i][7],
                                     u_values[i][8],
                                     u_values[i][9],
                                     get_total_thrust(t_values[i])[2]))
        libc2.updateRenderData()




def slider1_changed(value):
    update_rocket_mass(value)
    update_sim()
    update_graph2d()

axbox1 = fig2d.add_axes((0.5, 0.05, 0.1, 0.075))
slider_box1 = Slider(axbox1, "Mass", 0.5, 1.5, valinit=Rocket.rocket_mass)
slider_box1.on_changed(slider1_changed)

def slider2_changed(value):
    global wind_speed
    wind_speed = value
    update_sim()
    update_graph2d()

axbox2 = fig2d.add_axes((0.7, 0.05, 0.1, 0.075))
slider_box2 = Slider(axbox2, "Wind", 0.0, 5.0, valinit=wind_speed)
slider_box2.on_changed(slider2_changed)

def slider3_changed(value):
    global SECOND_IGNITION
    SECOND_IGNITION = value
    update_sim()
    update_graph2d()

axbox3 = fig2d.add_axes((0.3, 0.05, 0.1, 0.075))
slider_box3 = Slider(axbox3, "Ignition Time", BURN_TIME, 8.0, valinit=SECOND_IGNITION)
slider_box3.on_changed(slider3_changed)

update_graph2d()

fig3d = plt.figure()
ax3d = fig3d.add_subplot(projection='3d')
ax3d.plot(value_list_3d[0], value_list_3d[1], value_list_3d[2])

def runLibRenderer():
    libc2.nextFrame()

if useCPP_Renderer:
    renderer_deltaTime = int(1000.0 / 60.0)
    timer = fig2d.canvas.new_timer(renderer_deltaTime)

    timer.add_callback(runLibRenderer)
    timer.start()

plt.show()