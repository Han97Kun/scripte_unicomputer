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
import actionlib_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply
import numpy as np

from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint

node_prefix = 'raster_scan_demo: '
timeout = 10.0
t = 2  # Verweildauer der Sonde am Punkt (Sekunden)

# ---------------------------
# Kontext / MoveIt Einbindung
# ---------------------------
class Context:
    def __init__(self, move_group_name):
        rospy.loginfo(node_prefix + 'Waiting for move_group/status')
        rospy.wait_for_message('move_group/status', actionlib_msgs.msg.GoalStatusArray, timeout)

        self.commander = moveit_commander.MoveGroupCommander(move_group_name)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()   # wichtig fürs Retiming

        self.commander.set_end_effector_link('panda_sensor_mount')  # EE-Link
        self.commander.set_planning_time(10.0)                      # schlank für viele Punkte
        self.commander.set_num_planning_attempts(1)
        self.commander.set_planner_id("RRTConnectkConfigDefault")

        # einheitliche Limits (für plan() und Retiming)
        self.velocity_scaling = 0.10
        self.acceleration_scaling = 0.08
        self.commander.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.commander.set_max_acceleration_scaling_factor(self.acceleration_scaling)

        # Toleranzen etwas straffer (optional)
        self.commander.set_goal_position_tolerance(0.002)
        self.commander.set_goal_orientation_tolerance(0.02)

    def build_constraints(self, quat, joint7_angle=np.pi/4):
        # derzeit leer – falls du Constraints brauchst, hier aktivieren.
        constraints = Constraints()
        # Beispiel für künftige Nutzung (derzeit auskommentiert):
        """
        oc = OrientationConstraint()
        oc.header.frame_id = "panda_link0"
        oc.link_name = "panda_sensor_mount"
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = quat
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.11
        oc.absolute_z_axis_tolerance = 0.11
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        """
        return constraints

# -----------------------------------
# Safe-Pose (Gelenkraum) anfahren
# -----------------------------------
def move_to_safe_pose(commander):
    joint_goal = commander.get_current_joint_values()
    joint_goal[0] = 0.0                  # panda_joint1
    joint_goal[1] = -np.pi / 4           # panda_joint2
    joint_goal[2] = 0.0
    joint_goal[3] = -3*np.pi / 4
    joint_goal[4] = 0.0
    joint_goal[5] = np.pi / 2
    joint_goal[6] = np.pi / 4

    rospy.loginfo("Fahre in sichere Pose ...")
    commander.go(joint_goal, wait=True)
    commander.stop()
    rospy.loginfo("Sichere Gelenkpose erreicht.")

# --------------------------------------------------------
# Pose des Endeffektors für Sondenspitzen-Target berechnen
# --------------------------------------------------------
def compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length=0.1):
    rot_matrix = quaternion_matrix(quat)[:3, :3]
    offset_world = rot_matrix @ np.array([0, probe_length, 0])  # EE->Tip im Welt-Rahmen
    effector_position = probe_tip - offset_world

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = effector_position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
    return pose

# ------------------------
# Szene / Objekte aufbauen
# ------------------------
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

# ------------------------------------------------
# MoveIt-Planung zur Pose + Retiming (Hauptpfad)
# ------------------------------------------------
def plan_and_execute_pose(commander, ctx, pose_goal, maybe_constraints=None):
    # optional: Constraints setzen
    if maybe_constraints is not None:
        commander.set_path_constraints(maybe_constraints)

    commander.set_start_state_to_current_state()
    commander.set_pose_target(pose_goal)

    # EIN MoveIt-Befehl: planen + ausführen + zeitparametrisieren
    ok = commander.go(wait=True)

    commander.stop()
    commander.clear_pose_targets()
    commander.clear_path_constraints()
    return bool(ok), None


# -------------------------------------------------------------
# Fallback: Kartesische Bahn zum Ziel (ein Wegpunkt) + Retiming
# -------------------------------------------------------------
def cartesian_fallback(commander, ctx, pose_goal):
    commander.set_start_state_to_current_state()
    waypoints = [pose_goal]
    plan, fraction = commander.compute_cartesian_path(
        waypoints, eef_step=0.005, avoid_collisions=True, jump_threshold=0.0
    )

    if fraction < 0.999:
        return False, plan, fraction

    # Retiming
    # Retiming (korrekt: RobotState übergeben)
    try:
        plan = commander.retime_trajectory(
            ctx.robot.get_current_state(),  # <-- wichtig
            plan,
            velocity_scaling_factor=ctx.velocity_scaling,
            acceleration_scaling_factor=ctx.acceleration_scaling
        )
    except Exception as e:
        rospy.logwarn(f"Retiming fehlgeschlagen (cartesian): {e}")
        
    commander.execute(plan, wait=True)
    commander.stop()
    return True, plan, fraction

# ----------------
# Raster-Scan Loop
# ----------------
def raster_scan(ctx, config, marker_pub, box_pose, probe_length=0.1):
    # CSV vorbereiten
    results_dir = os.path.join(os.path.expanduser("~"), "catkin_ws", "raster_results")
    os.makedirs(results_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_file = os.path.join(results_dir, f"raster_log_{timestamp}.csv")
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z", "tilt_axis", "tilt_angle_deg",
                         "reachable", "fraction", "radial_distance",
                         "time_start", "time_end"])
    print(f"Schreibe Ergebnis-CSV nach: {output_file}")

    scene = ctx.scene
    commander = ctx.commander
    rospy.sleep(1)

    # Marker
    probe_marker = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD)
    probe_marker.scale.x = probe_marker.scale.y = probe_marker.scale.z = 0.02
    probe_marker.color.b = 1.0
    probe_marker.color.a = 1.0
    probe_marker.header.frame_id = "panda_link0"
    probe_marker.points = []

    # Basis-Orientierung der Sonde: Nach unten zeigend
    base_quat = quaternion_from_euler(-np.pi / 2, 0, 0)
    raster_size = config["raster_size"]
    delta = config["delta"]
    heights = config["heights"]
    tilt_angles_rad = [np.deg2rad(a) for a in config["tilt_angles_deg"]]

    x_start = box_pose.pose.position.x - ((raster_size - 1) / 2) * delta
    y_start = box_pose.pose.position.y - ((raster_size - 1) / 2) * delta

    idx = 0
    for z in heights:
        for i in range(raster_size):
            for j in range(raster_size):
                idx += 1
                probe_tip = np.array([x_start + i * delta, y_start + j * delta, z])
                tilt_axis = 'x'  # bei Bedarf wechseln

                for tilt_angle in tilt_angles_rad:
                    if tilt_axis == 'x':
                        tilt_quat = quaternion_from_euler(tilt_angle, 0, 0)
                    else:
                        tilt_quat = quaternion_from_euler(0, tilt_angle, 0)
                    quat = quaternion_multiply(tilt_quat, base_quat)
                    pose_goal = compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length)

                    # Optional: Constraints bauen/nutzen (hier standardmäßig None)
                    path_constraints = None
                    # path_constraints = ctx.build_constraints(quat)

                    # 1) Hauptweg: richtige MoveIt-Planung
                    ok, _traj = plan_and_execute_pose(commander, ctx, pose_goal, path_constraints)

                    tilt_deg = np.rad2deg(tilt_angle)

                    # Radialer Abstand zur Boxmitte (3D)
                    box_x = box_pose.pose.position.x
                    box_y = box_pose.pose.position.y
                    box_z = box_pose.pose.position.z
                    radial_dist = np.linalg.norm([
                        probe_tip[0] - box_x,
                        probe_tip[1] - box_y,
                        probe_tip[2] - box_z
                    ])

                    time_start = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    if ok:
                        # am Punkt verweilen
                        rospy.sleep(t)
                        time_end = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        # CSV: fraction = 1.0 (Planung erfolgreich)
                        with open(output_file, mode='a', newline='') as file:
                            writer = csv.writer(file)
                            writer.writerow([
                                probe_tip[0], probe_tip[1], probe_tip[2],
                                tilt_axis, tilt_deg,
                                True, 1.0,
                                round(radial_dist, 4),
                                time_start, time_end
                            ])
                        rospy.loginfo(f"[✓] Rasterpunkt {idx}: x={probe_tip[0]:.3f}, y={probe_tip[1]:.3f}, z={probe_tip[2]:.3f}, "
                                      f"Tilt-{tilt_axis}: {tilt_deg:.1f}° (MoveIt geplant)")
                        probe_marker.points.append(Point(*probe_tip))
                        marker_pub.publish(probe_marker)
                    else:
                        # 2) Fallback: kartesisch
                        ok2, _plan2, frac = cartesian_fallback(commander, ctx, pose_goal)
                        if ok2:
                            rospy.sleep(t)
                            time_end = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                            with open(output_file, mode='a', newline='') as file:
                                writer = csv.writer(file)
                                writer.writerow([
                                    probe_tip[0], probe_tip[1], probe_tip[2],
                                    tilt_axis, tilt_deg,
                                    True, round(frac, 2),
                                    round(radial_dist, 4),
                                    time_start, time_end
                                ])
                            rospy.loginfo(f"[~] Rasterpunkt {idx}: x={probe_tip[0]:.3f}, y={probe_tip[1]:.3f}, z={probe_tip[2]:.3f}, "
                                          f"Tilt-{tilt_axis}: {tilt_deg:.1f}° (kartesischer Fallback, fraction={frac:.2f})")
                            probe_marker.points.append(Point(*probe_tip))
                            marker_pub.publish(probe_marker)
                        else:
                            # nicht erreichbar → CSV ohne Zeiten
                            with open(output_file, mode='a', newline='') as file:
                                writer = csv.writer(file)
                                writer.writerow([
                                    probe_tip[0], probe_tip[1], probe_tip[2],
                                    tilt_axis, tilt_deg,
                                    False, round(frac, 2),
                                    round(radial_dist, 4),
                                    "", ""
                                ])
                            rospy.logwarn(f"[✗] Rasterpunkt {idx} NICHT erreichbar (fraction={frac:.2f})")

    rospy.loginfo("Fahre in sichere Pose zurück...")
    move_to_safe_pose(commander)

# -------------------
# Menü / Hauptschleife
# -------------------
def raster_menu(ctx):
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    box_pose = setup_environment(ctx.scene, ctx.commander)

    while not rospy.is_shutdown():
        print("\n=== Raster-Optionen ===")
        print("1: Standardraster (raster_size=3, heights=[0.02, 0.1, 0.2, 0.3], tilt_angles=[0°, 10°])")
        print("2: Einfaches Test-Raster um OCR zu testen (raster_size=2, heights=[0.02, 0.5], tilt_angles=[0°, +20°])")
        print("3: Einfaches Test-Raster (raster_size=1, heights=[0.3], tilt_angles=[0°])")
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
                    "raster_size": 2,
                    "delta": 0.05,
                    "heights": [0.1, 0.2, 0.3],
                    "tilt_angles_deg": [0, 10]
                }
            elif raster_choice == '2':
                config = {
                    "raster_size": 3,
                    "delta": 0.05,
                    "heights": [0.2, 0.3],
                    "tilt_angles_deg": [0, 10, 20]
                }
            else:  # '3'
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

# -----------
# main
# -----------
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('raster_scan_demo')

    ctx = Context('panda_arm')
    rospy.loginfo(f"hasattr(ctx,'robot') = {hasattr(ctx,'robot')}")
    raster_menu(ctx)
