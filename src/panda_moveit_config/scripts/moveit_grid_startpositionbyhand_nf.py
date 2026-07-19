#!/usr/bin/env python3
import sys
import rospy
import copy
import moveit_commander
import geometry_msgs.msg
import csv
import os
import time
from datetime import datetime
#import controller_manager_msgs.srv
import actionlib
import actionlib_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply
import numpy as np

from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint, DisplayTrajectory
from actionlib_msgs.msg import GoalStatusArray

node_prefix = 'raster_scan_demo: '
timeout = 10.0
t = 1  # Verweildauer der Sonde am Punkt (s)

# einige links: https://projects.saifsidhik.page/franka_ros_interface/DOC.html#module-franka_moveit


class Context:
    def __init__(self, move_group_name):
        rospy.loginfo(node_prefix + 'Waiting for move_group/status')
        rospy.wait_for_message('move_group/status', actionlib_msgs.msg.GoalStatusArray, timeout)

        self.commander = moveit_commander.MoveGroupCommander(move_group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

        self.commander.set_end_effector_link('panda_sensor_mount')  # panda_link8
        self.commander.set_planning_time(30)
        self.commander.set_num_planning_attempts(3)

        # langsamer fahren
        self.commander.set_max_velocity_scaling_factor(0.05)
        self.commander.set_max_acceleration_scaling_factor(0.03)

    def build_constraints(self, quat, joint7_angle=np.pi / 4):
        constraints = Constraints()
        """
        oc = OrientationConstraint()
        oc.header.frame_id = "panda_link0"
        oc.link_name = "panda_link8"
        oc.orientation.x = quat[0]
        oc.orientation.y = quat[1]
        oc.orientation.z = quat[2]
        oc.orientation.w = quat[3]
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.11
        oc.absolute_z_axis_tolerance = 0.11
        oc.weight = 1.0
        #constraints.orientation_constraints.append(oc)

        jc = JointConstraint()
        jc.joint_name = "panda_joint7"
        jc.position = joint7_angle
        jc.tolerance_above = 0.1
        jc.tolerance_below = 0.1
        jc.weight = 1.0
        #constraints.joint_constraints.append(jc)
        """
        # PositionConstraint (nur Y >= 0.0 zulässig)
        pc = PositionConstraint()
        pc.header.frame_id = "panda_link0"
        pc.link_name = "panda_sensor_mount"
        pc.weight = 1.0

        # Definiere ein Box-Primitiv, das nur Y >= 0 erlaubt
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [3.0, 3.0, 3.0]  # [x, y, z] Größe

        # Setze Ursprung des erlaubten Bereichs (Box Mitte)
        pose = PoseStamped()
        pose.header.frame_id = "panda_link0"
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.5  # Box geht von y=-1.0 bis y=2 (Mitte bei y=1)
        pose.pose.position.z = 0.0

        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(pose.pose)

        #constraints.position_constraints.append(pc)

        return constraints

def move_to_safe_pose(commander):
    # aktuelle Gelenkwinkel holen
    joint_goal = commander.get_current_joint_values()

    # feste Gelenkwinkel setzen
    joint_goal[0] = 0.0                  # panda_joint1
    joint_goal[1] = -np.pi / 4           # panda_joint2
    joint_goal[2] = 0.0
    joint_goal[3] = -np.pi*3 / 4         # 135°
    joint_goal[4] = 0.0
    joint_goal[5] = np.pi / 2            # 90°
    joint_goal[6] = np.pi / 4            # 45°

    rospy.loginfo("Fahre in sichere Pose ...")
    commander.go(joint_goal, wait=True)
    commander.stop()
    rospy.loginfo("Sichere Gelenkpose erreicht.")

def compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length=0.1):
    rot_matrix = quaternion_matrix(quat)[:3, :3] #orientierung edeffektor
    #print(f"rot_matrix: {rot_matrix}") 

    offset_world = rot_matrix @ np.array([0, probe_length, 0]) # KOS von Sonde verschieden #vektor vom endeffektor zur sondenpsitze

    effector_position = probe_tip - offset_world # probe _tip (Zielposition der Sondenspitze) um position des endeffektors zu berechnen

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = effector_position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat

    return pose

def setup_environment(scene, commander):
    rospy.sleep(2)
    table_pose = PoseStamped()
    table_pose.header.frame_id = commander.get_planning_frame()
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.x = 0.3
    table_pose.pose.position.z = -0.0251
    scene.add_box("table", table_pose, (0.9, 0.9, 0.05))

    wall_pose = PoseStamped()
    wall_pose.header.frame_id = commander.get_planning_frame()
    wall_pose.pose.orientation.w = 1.0
    wall_pose.pose.position.x = -1
    wall_pose.pose.position.z = 0.5
    scene.add_box("wall", wall_pose, (0.005, 2.0, 2.0))

    legs_size = (0.05, 0.05, 0.4)
    for idx, (x, y, z) in enumerate([
        (-0.125, 0.425, -0.225),
        (-0.125, -0.425, -0.225),
        (0.725, 0.425, -0.225),
        (0.725, -0.425, -0.225)], 1):
        leg_pose = PoseStamped()
        leg_pose.header.frame_id = commander.get_planning_frame()
        leg_pose.pose.orientation.w = 1.0
        leg_pose.pose.position.x = x
        leg_pose.pose.position.y = y
        leg_pose.pose.position.z = z
        scene.add_box(f"leg{idx}", leg_pose, legs_size)

    box_pose = PoseStamped()
    box_pose.header.frame_id = commander.get_planning_frame()
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.325
    box_pose.pose.position.z = 0.005
    scene.add_box("small_box", box_pose, (0.05, 0.03, 0.01))

    rospy.sleep(2)
    return box_pose


def select_manual_origin(scene, commander, probe_length=0.1, marker_radius=0.015):
    """
    Nutzereingriff: Panda per Hand an den gewünschten Ursprung fahren (Sondenspitze an Ziel),
    Enter drücken -> dieser Punkt wird als Raster-Mitte (box_pose) verwendet.
    Zudem wird 'small_box' an dieser Position im Planning Scene hinzugefügt/verschoben.
    """
    input("\n[Manuell] Führe die Sonde (Handführung) an die gewünschte Raster-Mitte (Sondenspitze) und drücke <Enter> ...")

    # aktuelle EEF-Pose
    cur = commander.get_current_pose().pose
    q = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w])
    R = quaternion_matrix(q)[:3, :3]
    offset_world = R @ np.array([0, probe_length, 0])

    eef = np.array([cur.position.x, cur.position.y, cur.position.z])
    tip = eef + offset_world  # inverse von compute_effector_pose_for_probe_tip

    rospy.loginfo(f"[Manuell] Referenz (TIP) gesetzt auf: x={tip[0]:.3f}, y={tip[1]:.3f}, z={tip[2]:.3f}")

    # Box/Marker an diese Stelle in die Szene setzen (als kleiner Würfel oder Kugel)
    planning_frame = commander.get_planning_frame()
    try:
        scene.remove_world_object("small_box")
        rospy.sleep(0.2)
    except Exception:
        pass

    box_pose = PoseStamped()
    box_pose.header.frame_id = planning_frame
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = tip[0]
    box_pose.pose.position.y = tip[1]
    box_pose.pose.position.z = tip[2]

    # kleiner Marker – hier: Würfel 3 cm Kantenlänge
    scene.add_box("small_box", box_pose, (0.03, 0.03, 0.03))
    rospy.sleep(0.5)

    return box_pose  # enthält die TIP-Position als Ursprung


def raster_scan(ctx, config, marker_pub, box_pose, probe_length=0.1):
    # CSV vorbereiten
    results_dir = os.path.join(os.path.expanduser("~"), "catkin_ws", "raster_results")
    os.makedirs(results_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_file = os.path.join(results_dir, f"raster_log_{timestamp}.csv")
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z", "tilt_axis", "tilt_angle_deg", "reachable", "fraction", "radial_distance", "time_start", "time_end"])

    print(f"Schreibe Ergebnis-CSV nach: {output_file}")

    scene = ctx.scene
    commander = ctx.commander
    rospy.sleep(0.5)

    # Marker für RViz
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    probe_marker = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD)
    probe_marker.scale.x = probe_marker.scale.y = probe_marker.scale.z = 0.02
    probe_marker.color.b = 1.0
    probe_marker.color.a = 1.0
    probe_marker.header.frame_id = "panda_link0"
    probe_marker.points = []

    # Basis-Orientierung der Sonde: Nach unten zeigend (Werkzeug-y Richtung Probe)
    base_quat = quaternion_from_euler(-np.pi / 2, 0, 0)

    raster_size = config["raster_size"]
    delta = config["delta"]
    heights = config["heights"]
    tilt_angles_rad = [np.deg2rad(a) for a in config["tilt_angles_deg"]]

    # Raster um die per Hand gesetzte box_pose (TIP)
    x_start = box_pose.pose.position.x - ((raster_size - 1) / 2.0) * delta
    y_start = box_pose.pose.position.y - ((raster_size - 1) / 2.0) * delta

    total = len(heights) * raster_size**2
    idx = 0
    for z in heights:
        for i in range(raster_size):
            for j in range(raster_size):
                idx += 1
                probe_tip = np.array([x_start + i * delta, y_start + j * delta, z])  # Zielposition (TIP)
                tilt_axis = 'x'  # wie in deinem Skript aktuell festgelegt

                for tilt_angle in tilt_angles_rad:
                    if tilt_axis == 'x':
                        tilt_quat = quaternion_from_euler(tilt_angle, 0, 0)
                    else:
                        tilt_quat = quaternion_from_euler(0, tilt_angle, 0)

                    quat = quaternion_multiply(tilt_quat, base_quat)
                    pose_goal = compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length)

                    # planen
                    commander.set_path_constraints(ctx.build_constraints(quat))
                    commander.set_start_state_to_current_state()
                    plan, fraction = commander.compute_cartesian_path(
                        [pose_goal], eef_step=0.005, avoid_collisions=True, jump_threshold=0.0
                    )
                    commander.clear_path_constraints()

                    tilt_deg = np.rad2deg(tilt_angle)

                    if fraction > 0.8 and plan and plan.joint_trajectory.points:
                        # Distanz zur Referenz (manuell gesetzter TIP)
                        box_x = box_pose.pose.position.x
                        box_y = box_pose.pose.position.y
                        box_z = box_pose.pose.position.z
                        radial_dist = np.linalg.norm([probe_tip[0] - box_x,
                                                      probe_tip[1] - box_y,
                                                      probe_tip[2] - box_z])

                        start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        ok = commander.execute(plan, wait=True)
                        commander.stop()
                        rospy.sleep(t)
                        end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                        with open(output_file, mode='a', newline='') as file:
                            writer = csv.writer(file)
                            writer.writerow([
                                probe_tip[0], probe_tip[1], probe_tip[2],
                                tilt_axis, tilt_deg,
                                True, round(fraction, 2),
                                round(radial_dist, 4),
                                start_time, end_time
                            ])

                        rospy.loginfo(f"[✓] Rasterpunkt {idx}: x={probe_tip[0]:.3f}, y={probe_tip[1]:.3f}, z={probe_tip[2]:.3f}, "
                                      f"Tilt-{tilt_axis}: {tilt_deg:.1f}° (fraction={fraction:.2f}) | ok={ok}")
                        probe_marker.points.append(Point(*probe_tip))
                        marker_pub.publish(probe_marker)
                    else:
                        # Unreachable / schlechte fraction
                        box_x = box_pose.pose.position.x
                        box_y = box_pose.pose.position.y
                        box_z = box_pose.pose.position.z
                        radial_dist = np.linalg.norm([probe_tip[0] - box_x,
                                                      probe_tip[1] - box_y,
                                                      probe_tip[2] - box_z])

                        with open(output_file, mode='a', newline='') as file:
                            writer = csv.writer(file)
                            writer.writerow([
                                probe_tip[0], probe_tip[1], probe_tip[2],
                                tilt_axis, tilt_deg,
                                False, round(fraction, 2),
                                round(radial_dist, 4),
                                "", ""
                            ])

                        rospy.logwarn(f"[✗] Rasterpunkt {idx} NICHT erreichbar (fraction={fraction:.2f})")

    rospy.loginfo("Fahre in sichere Pose zurück...")
    move_to_safe_pose(commander)


def raster_menu(ctx):
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    setup_environment(ctx.scene, ctx.commander)  # Umgebung ohne Quelle

    while not rospy.is_shutdown():
        print("\n=== Raster-Optionen ===")
        print("1: Standardraster (raster_size=3, heights=[0.02, 0.1, 0.2, 0.3], tilt_angles=[0°, 10°])")
        print("2: Einfaches Test-Raster (raster_size=2, heights=[0.2, 0.3], tilt_angles=[0°, 10°])")
        print("3: Einfaches Test-Raster (raster_size=1, heights=[0.1], tilt_angles=[0°])")
        print("q: Beenden")

        raster_choice = input("Wähle Raster-Option (1/2/3/q): ").strip().lower()
        if raster_choice == 'q':
            print("Beende Raster-Demo.")
            break
        if raster_choice not in ['1', '2', '3']:
            print("Ungültige Eingabe. Bitte erneut versuchen.")
            continue

        # **HIER**: Manuelle Ursprungsauswahl (Sondenspitze)
        box_pose = select_manual_origin(ctx.scene, ctx.commander, probe_length=0.1)

        repeat = True
        while repeat and not rospy.is_shutdown():
            if raster_choice == '1':
                config = {
                    "raster_size": 2,
                    "delta": 0.05,
                    "heights": [0.1, 0.2, 0.3],
                    "tilt_angles_deg": [0, 10]
                }
            elif raster_choice == '2':
                config = {
                    "raster_size": 14,
                    "delta": 0.01,
                    "heights": [0.2, 0.3],
                    "tilt_angles_deg": [0, 10]
                }
            else:  # '3'
                config = {
                    "raster_size": 1,
                    "delta": 0.05,
                    "heights": [0.1],
                    "tilt_angles_deg": [0]
                }

            raster_scan(ctx, config, marker_pub, box_pose)

            answer = input("\nNochmal ausführen (j/n)? ").strip().lower()
            if answer != 'j':
                repeat = False


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('raster_scan_demo')

    ctx = Context('panda_arm')  # deine MoveIt-Gruppe
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    raster_menu(ctx)
