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
import actionlib
import actionlib_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply
import numpy as np
from moveit_msgs.msg import Constraints  # Hier fügen wir den fehlenden Import hinzu
from actionlib_msgs.msg import GoalStatusArray

 

from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import PositionConstraint

 

'''
Test hier die ganze trajektorie zu planen und mit pause auszuführen - klappt nicht 
'''

 

node_prefix = 'raster_scan_demo: '
timeout = 10.0
t = 5  # Verweildauer der Sonde am Punkt, dieser Wert könnte angepasst werden

 

class Context:
    def __init__(self, move_group_name):
        rospy.loginfo(node_prefix + 'Waiting for move_group/status')
        rospy.wait_for_message('move_group/status', actionlib_msgs.msg.GoalStatusArray, timeout)

 

        self.commander = moveit_commander.MoveGroupCommander(move_group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

 

        self.commander.set_end_effector_link('panda_sensor_mount')  # panda_link8
        self.commander.set_planning_time(30)  # 30 Sekunden
        self.commander.set_num_planning_attempts(3)  # 3 Versuche

        self.commander.set_max_velocity_scaling_factor(0.2)
        self.commander.set_max_acceleration_scaling_factor(0.2)


 

    def build_constraints(self, quat, joint7_angle=np.pi / 4):
        constraints = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "panda_link0"
        pc.link_name = "panda_sensor_mount"
        pc.weight = 1.0

 

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [3.0, 3.0, 3.0]  # [x, y, z] Größe

 

        pose = PoseStamped()
        pose.header.frame_id = "panda_link0"
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.5  # Box geht von y=-1.0 bis y=2 (Mitte bei y=1)
        pose.pose.position.z = 0.0

 

        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(pose.pose)

 

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
    rot_matrix = quaternion_matrix(quat)[:3, :3]  # Orientierung Endeffektor
    offset_world = rot_matrix @ np.array([0, probe_length, 0])  # KOS von Sonde verschieden
    effector_position = probe_tip - offset_world  # Probe-Tip (Zielposition der Sondenspitze)

 

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

 

    box_pose = PoseStamped()
    box_pose.header.frame_id = commander.get_planning_frame()
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.325
    box_pose.pose.position.z = 0.005
    scene.add_box("small_box", box_pose, (0.05, 0.03, 0.01))

 

    rospy.sleep(2)
    return box_pose

 

def raster_scan(ctx, config, marker_pub, box_pose, probe_length=0.1):
    results_dir = os.path.join(os.path.expanduser("~"), "catkin_ws", "raster_results")
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")  # datetime
    output_file = os.path.join(results_dir, f"raster_log_{timestamp}.csv")

 

    # Datei initialisieren / in Header schreiben
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z", "tilt_axis", "tilt_angle_deg", "reachable", "fraction", "radial_distance", "time_start", "time_end"])

 

    print(f"Schreibe Ergebnis-CSV nach: {output_file}")

 

    scene = ctx.scene
    commander = ctx.commander

 

    rospy.sleep(2)

 

    probe_marker = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD)
    probe_marker.scale.x = probe_marker.scale.y = probe_marker.scale.z = 0.02
    probe_marker.color.b = 1.0
    probe_marker.color.a = 1.0
    probe_marker.header.frame_id = "panda_link0"
    probe_marker.points = []

 

    base_quat = quaternion_from_euler(-np.pi / 2, 0, 0)
    raster_size = config["raster_size"]
    delta = config["delta"]
    heights = config["heights"]
    tilt_angles_rad = [np.deg2rad(a) for a in config["tilt_angles_deg"]]

 

    x_start = box_pose.pose.position.x - ((raster_size - 1) / 2) * delta
    y_start = box_pose.pose.position.y - ((raster_size - 1) / 2) * delta

 

    waypoints = []  # Liste für Wegpunkte
    helfer = []
    idx = 0

 

    for z in heights:
        for i in range(raster_size):
            index =  range(raster_size) if (i % 2 == 0) else range(raster_size-1, -1, -1)
            for j in index:
                idx += 1
                probe_tip = np.array([x_start + i * delta, y_start + j * delta, z])
                tilt_axis = 'x' if (idx % 2 == 1) else 'y'

 

                for tilt_angle in tilt_angles_rad:
                    if tilt_axis == 'x':
                        tilt_quat = quaternion_from_euler(tilt_angle, 0, 0)
                    else:
                        tilt_quat = quaternion_from_euler(0, tilt_angle, 0)
                    
                    quat = quaternion_multiply(tilt_quat, base_quat)
                    pose_goal = compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length)
                    waypoints.append(pose_goal)

 

                    tilt_deg = float(np.rad2deg(tilt_angle))
                    helfer.append((probe_tip.copy(), tilt_axis, tilt_deg)) 
                    

 

    # Berechne den gesamten Cartesian Path
    commander.set_start_state_to_current_state()
    commander.set_path_constraints(ctx.build_constraints(quat))
    plan, fraction = commander.compute_cartesian_path(
        waypoints, eef_step=0.005, avoid_collisions=True,jump_threshold = 0.0
    )
    rospy.loginfo(f"[plan-all] waypoints={len(waypoints)}, fraction={fraction:.2f}")

 

    # Führe die Trajektorie aus
    success = commander.execute(plan, wait=True)
    if success:
        rospy.loginfo("[✓] Trajektorie erfolgreich ausgeführt.")
    else:
        rospy.logwarn("[✗] Fehler bei der Trajektoorie.")
    for idx, (waypoint, (probe_tip, axis, tilt_deg)) in enumerate(zip(waypoints, helfer)):
        radial_dist = np.linalg.norm([
            probe_tip[0] - box_pose.pose.position.x,
            probe_tip[1] - box_pose.pose.position.y,
            probe_tip[2] - box_pose.pose.position.z
        ])
        
        start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        #rospy.loginfo(f"[✓] Rasterpunkt {idx}: x={waypoint.position.x:.3f}, y={waypoint.position.y:.3f}, z={waypoint.position.z:.3f}")
        rospy.loginfo(
        f"[✓] Rasterpunkt {idx}: TIP=({probe_tip[0]:.3f}, {probe_tip[1]:.3f}, {probe_tip[2]:.3f}) | "
        f"EEF=({waypoint.position.x:.3f}, {waypoint.position.y:.3f}, {waypoint.position.z:.3f}) | "
        f"Tilt-{tilt_axis}: {tilt_deg:.1f}°"
        f"| fraction_all={fraction:.2f}"
        )
        #tilt_deg = np.rad2deg(tilt_angle)

 

        probe_marker.points.append(Point(probe_tip[0], probe_tip[1], probe_tip[2]))
        marker_pub.publish(probe_marker)

 

        # Die Wartezeit ist nun in den Wegpunkten integriert, daher keine extra sleep erforderlich
        end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        with open(output_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                probe_tip[0], probe_tip[1], probe_tip[2],
                tilt_axis, np.rad2deg(tilt_angle), fraction > 0.8, round(fraction, 2),
                round(radial_dist, 4), start_time, end_time
            ])

 

        probe_marker.points.append(Point( probe_tip[0], probe_tip[1],  probe_tip[2]))

 

        marker_pub.publish(probe_marker)

 

    rospy.loginfo("Fahre in sichere Pose zurück...")
    move_to_safe_pose(commander)

 

def raster_menu(ctx):
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    box_pose = setup_environment(ctx.scene, ctx.commander)

 

    while not rospy.is_shutdown():
        print("\n=== Raster-Optionen ===")
        print("1: Standardraster (raster_size=5, heights=[0.02, ...], tilt_angles=[±20°, ±40°])")
        print("2: Einfaches Test-Raster um OCR zu testen (raster_size=2, heights=[0.02, 2], tilt_angles=[0, +20°])")
        print("3: Einfaches Test-Raster (raster_size=1, heights=[0.05], tilt_angles=[+20°])")
        print("q: Beenden")

 

        raster_choice = input("Wähle Raster-Option (1/2/3/q): ").strip().lower()

 

        if raster_choice == 'q':
            print("Beende Raster-Demo.")
            break

 

        if raster_choice not in ['1', '2', '3']:
            print("Ungültige Eingabe. Bitte erneut versuchen.")
            continue

 

        repeat = True
        while repeat and not rospy.is_shutdown():
            if raster_choice == '1':
                config = {
                    "raster_size": 5,
                    "delta": 0.05,
                    "heights": [0.02, 0.05, 0.1, 0.15, 0.2],
                    "tilt_angles_deg": [0, 20, 40, -20, -40]
                }

 

            if raster_choice == '2':
                config = {
                    "raster_size": 2,
                    "delta": 0.05,
                    "heights": [0.2],
                    "tilt_angles_deg": [0, 20, -20]
                }

 

            elif raster_choice == '3':
                config = {
                    "raster_size": 1,
                    "delta": 0.05,
                    "heights": [0.3],
                    "tilt_angles_deg": [0]
                }

 

            raster_scan(ctx, config, marker_pub, box_pose)

 

            answer = input("\nNochmal ausführen (j/n)? ").strip().lower()
            if answer != 'j':
                repeat = False

 

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('raster_scan_demo')
    
    ctx = Context('panda_arm')  # moveit Gruppe
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    raster_menu(ctx)
