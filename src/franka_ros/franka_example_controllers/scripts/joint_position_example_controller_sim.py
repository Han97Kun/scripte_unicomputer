#!/usr/bin/env python3
"""
import rospy
from std_msgs.msg import Float64MultiArray
def main():
   rospy.init_node('simple_trajectory_sender')
   pub = rospy.Publisher(
       '/joint_position_example_controller_sim/joint_trajectory_command',
       Float64MultiArray,
       queue_size=10
   )
   rospy.sleep(1)  # Warten, bis Publisher bereit

   # Liste von Positionen, die nacheinander gesendet werden
   positions = [
       [0.0, -0.3, 0.0, -2.0, 0.0, 1.7, 0.8],
       [0.4, -0.5, 0.2, -2.2, 0.1, 1.8, 1.0],
       [-0.3, -0.6, 0.1, -1.9, 0.0, 1.4, 1.2]
   ]
   rate = rospy.Rate(0.3)  # ca. alle 3 Sekunden eine Position senden
   for pos in positions:
       msg = Float64MultiArray()
       msg.data = pos
       rospy.loginfo(f"Sende Position: {pos}")
       pub.publish(msg)
       rate.sleep()
   rospy.loginfo("Fertig.")

 

if __name__ == '__main__':
   main()
"""

"""
#!/usr/bin/env python3
import rospy, numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock  # nur wenn use_sim_time=true

TOPIC = "/joint_position_example_controller_sim/joint_trajectory_command"  # ggf. mit rostopic info prüfen
RATE_HZ = 200.0       # hoch halten, damit keine Sprünge entstehen
SEGMENT_TIME = 3.0    # s pro Segment

def wait_for_subscriber(pub, timeout=5.0):
    t0 = rospy.Time.now()
    while pub.get_num_connections() == 0 and (rospy.Time.now() - t0).to_sec() < timeout:
        rospy.sleep(0.05)
    return pub.get_num_connections() > 0

def get_current_q():
    # In der Sim publisht Gazebo auf /joint_states
    js = rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
    return list(js.position[:7])

def stream_to(pub, q_start, q_goal, T=SEGMENT_TIME, rate_hz=RATE_HZ):
    rate = rospy.Rate(rate_hz)
    steps = max(1, int(T * rate_hz))
    q_start = np.array(q_start, dtype=float)
    q_goal  = np.array(q_goal,  dtype=float)
    for i in range(1, steps + 1):
        if rospy.is_shutdown(): break
        a = float(i) / steps
        q = (1.0 - a) * q_start + a * q_goal
        pub.publish(Float64MultiArray(data=q.tolist()))
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("sim_trajectory_streamer")

    # falls use_sim_time:=true → auf /clock warten, sonst läuft Rate nicht
    if rospy.get_param("/use_sim_time", False):
        rospy.loginfo("Wartee auf /clock (use_sim_time=true) ...")
        rospy.wait_for_message("/clock", Clock)

    pub = rospy.Publisher(TOPIC, Float64MultiArray, queue_size=10)
    if not wait_for_subscriber(pub, timeout=5.0):
        rospy.logerr(f"Kein Subscriber auf {TOPIC}. Prüfe Controller/Topic-Name mit: rostopic info {TOPIC}")
        raise SystemExit(1)

    # Ziele definieren
    targets = [
        [0.0, -0.3, 0.0, -2.0, 0.0, 1.7, 0.8],
        [0.4, -0.5, 0.2, -2.2, 0.1, 1.8, 1.0],
        [-0.3, -0.6, 0.1, -1.9, 0.0, 1.4, 1.2],
    ]

    # Sanft vom Istzustand starten
    q_curr = get_current_q()
    rospy.loginfo(f"Starte von aktueller Pose: {np.round(q_curr,3).tolist()}")

    stream_to(pub, q_curr, targets[0])
    for a, b in zip(targets, targets[1:]):
        stream_to(pub, a, b)

    rospy.loginfo("Fertig.")
"""
#!/usr/bin/env python3
import sys, rospy, numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

TOPIC = "/joint_position_example_controller_sim/joint_position"  # GENAU wie in rosnode info!
MAX_VEL = np.array([0.15,0.15,0.15,0.2,0.2,0.25,0.25])
EPS = 1e-4

def q_now():
    try:
        js = rospy.wait_for_message("/joint_states", JointState, timeout=3.0)
        return np.array(js.position[:7], float)
    except rospy.ROSException:
        rospy.logerr("Kein /joint_states – läuft Gazebo? joint_state_controller/robot_state_publisher?")
        sys.exit(1)

if __name__ == "__main__":
    rospy.init_node("one_move_stream")

    pub = rospy.Publisher(TOPIC, Float64MultiArray, queue_size=20)
    # Auf Subscriber warten (max 8s)
    t0 = rospy.Time.now()
    while pub.get_num_connections() == 0 and (rospy.Time.now()-t0).to_sec() < 8.0:
        rospy.sleep(0.05)
    if pub.get_num_connections() == 0:
        rospy.logerr("Kein Subscriber auf %s. Prüfe mit:\n  rosnode info /joint_position_example_controller_sim", TOPIC)
        sys.exit(1)

    q = q_now()
    q_goal = np.array([0.0,-0.3,0.0,-2.0,0.0,1.7,0.8], float)

    last_t = rospy.Time.now()
    sent = 0
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last_t).to_sec()
        if dt <= 0.0:
            rospy.sleep(0.0005); continue
        last_t = now

        err = q_goal - q
        if np.all(np.abs(err) < EPS):
            break

        step = np.clip(err, -MAX_VEL*dt*0.8, MAX_VEL*dt*0.8)
        q = q + step
        pub.publish(Float64MultiArray(data=q.tolist()))
        sent += 1

        # kleine kooperative Pause (Timing macht dt)
        rospy.sleep(0.0005)

    rospy.loginfo("Fertig. Gesendete Schritte: %d", sent)
