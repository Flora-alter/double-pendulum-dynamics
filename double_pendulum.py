import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anime
import math


def double_pendulum_dynamics(q_1, q_2, q_1d, q_2d, tau_1, tau_2):
    a = m_2 * l_1 * l_g2 * math.cos(q_2)
    b = m_2 * l_g2 * l_g2 + i_2
    c = m_2 * l_1 * l_g2 * q_2d * math.sin(q_2)
    d = m_2 * g * l_g2 * math.cos(q_1 + q_2)
    e = m_2 * l_1 * l_g2 * q_1d * math.sin(q_2)
    M = np.array([
        [m_1*l_g1*l_g1 + m_2*l_1*l_1 + i_1 + 2*a + b, a + b],
        [a + b                                      , b    ]
    ])
    M_d = np.array([
        [-2*c, -c],
        [-c  ,  0]
    ])
    dKdq = np.array([0, -e*(q_1d + q_2d)])
    dUdq = np.array([m_1*g*l_g1*math.cos(q_1) + m_2*g*l_1*math.cos(q_1) + d, d])
    dQdt = np.array([q_1d, q_2d])
    Tau = np.array([tau_1, tau_2])
    q_dd = np.linalg.inv(M) @ (Tau - M_d @ dQdt + dKdq - dUdq)
    return q_dd


def draw_simulator(t, ax, q_1_list, q_2_list):
    q_1 = q_1_list[t]
    q_2 = q_2_list[t]
    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim([-(l_1 + l_2 + 1.0), (l_1 + l_2 + 1.0)])
    ax.set_ylim([-(l_1 + l_2 + 1.0), (l_1 + l_2 + 1.0)])
    ax.set_title(f"2-link robot t={t*dt:.4}")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid()
    elbow_joint_x = l_1 * math.cos(q_1)
    elbow_joint_y = l_1 * math.sin(q_1)
    endpoint_x = l_1 * math.cos(q_1) + l_2 * math.cos(q_1 + q_2)
    endpoint_y = l_1 * math.sin(q_1) + l_2 * math.sin(q_1 + q_2)
    ax.scatter(0, 0, s=20, color="green")
    ax.plot([0, elbow_joint_x], [0, elbow_joint_y], color="blue")
    ax.scatter(elbow_joint_x, elbow_joint_y, s=20, color="red")
    ax.plot([elbow_joint_x, endpoint_x], [elbow_joint_y, endpoint_y], color="blue")
    ax.scatter(endpoint_x, endpoint_y, s=20, color="red")


def main(q_1_init, q_1d_init, q_2_init, q_2d_init):
    t = 0.0
    q_d = np.array([q_1d_init, q_2d_init])
    q = np.array([q_1_init, q_2_init])
    tau_1 = 0.0
    tau_2 = 0.0
    q_1_list = []
    q_2_list = []

    while t < T:
        q_1 = q[0]
        q_2 = q[1]
        q_1d = q_d[0]
        q_2d = q_d[1]
        q_dd = double_pendulum_dynamics(q_1, q_2, q_1d, q_2d, tau_1, tau_2)
        q_d = q_d + q_dd * dt
        q = q + q_d * dt
        q_1_list.append(q_1)
        q_2_list.append(q_2)
        t += dt
    
    fig, ax = plt.subplots()
    ani = anime.FuncAnimation(fig, draw_simulator, fargs=(ax, q_1_list, q_2_list), frames=int(T/dt) + 1, interval=int(dt*1000))
    plt.show()


l_1 = 1.0
l_g1 = l_1 / 2
m_1 = 2.0
i_1 = 0.5

l_2 = 1.0
l_g2 = l_1 / 2
m_2 = 1.0
i_2 = 0.2

g = 9.8
# simulation period
T = 5.0
dt = 0.01

# initial state
q_1 = 2.0 * math.pi / 3.0
q_2 = math.pi / 3.0
q_1d = 0.0
q_2d = 0.0


if __name__ == "__main__":
    main(q_1, q_1d, q_2, q_2d)