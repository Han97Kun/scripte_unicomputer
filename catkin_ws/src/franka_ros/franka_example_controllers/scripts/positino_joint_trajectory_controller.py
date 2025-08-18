#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

JOINTS = ["panda_joint1","panda_joint2","panda_joint3",
          "panda_joint4","panda_joint5","panda_joint6","panda_joint7"]
CMD_TOPIC = "/position_joint_trajectory_controller/command"

def get_q_current():
    js = rospy.wait_for_message("/joint_states", JointState, timeout=3.0)
    # Annahme: Reihenfolge passt (bei Franka üblich)
    return list(js.position[:7])

def send_trajectory(targets, seg_time=3.0, settle_time=0.5):
    pub = rospy.Publisher(CMD_TOPIC, JointTrajectory, queue_size=10)
    rospy.sleep(0.3)  # Publisher-Handshake

    traj = JointTrajectory(joint_names=JOINTS)

    # Punkt 0 = aktueller Zustand (0 s), damit keine Sprünge entstehen
    q0 = get_q_current()
    traj.points.append(JointTrajectoryPoint(
        positions=q0, velocities=[0.0]*7, accelerations=[0.0]*7,
        time_from_start=rospy.Duration(0.0)
    ))

    t = settle_time  # kleine Haltezeit, damit Controller „fassen“ kann
    for q in targets:
        traj.points.append(JointTrajectoryPoint(
            positions=q,
            velocities=[0.0]*7,            # Endgeschw. 0 → sanftes Ausrollen
            accelerations=[0.0]*7,         # Endbeschl. 0
            time_from_start=rospy.Duration.from_sec(t)
        ))
        t += seg_time

    pub.publish(traj)
    rospy.loginfo("Trajektorie gesendet.")

if __name__ == "__main__":
    rospy.init_node("panda_traj_sender")

    targets = [
        [0.0, -0.3, 0.0, -2.0, 0.0, 1.7, 0.8],
        [0.4, -0.5, 0.2, -2.2, 0.1, 1.8, 1.0],
        [-0.3,-0.6, 0.1, -1.9, 0.0, 1.4, 1.2],
    ]
    send_trajectory(targets, seg_time=3.0, settle_time=0.5)
