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
t = 2  #verweildauer der sonde am punkt

#eingie links: https://projects.saifsidhik.page/franka_ros_interface/DOC.html#module-franka_moveit


class Context:
    def __init__(self, move_group_name):
        rospy.loginfo(node_prefix + 'Waiting for move_group/status')
        rospy.wait_for_message('move_group/status', actionlib_msgs.msg.GoalStatusArray, timeout)

        self.commander = moveit_commander.MoveGroupCommander(move_group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

        self.commander.set_end_effector_link('panda_sensor_mount') #panda_link8
        self.commander.set_planning_time(30) #30
        self.commander.set_num_planning_attempts(3) #5

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
    commander.clear_path_constraints()
    commander.clear_pose_targets()
    commander.set_start_state_to_current_state()
    ok = commander.go(joint_goal, wait=True)
    commander.stop()
    if ok:
        rospy.loginfo("Sichere Gelenkpose erreicht.")
        return True

    # Fallback: plan+execute im Joint-Space
    try:
        commander.set_start_state_to_current_state()
        commander.set_joint_value_target(joint_goal)
        plan = commander.plan()
        traj = plan[1] if isinstance(plan, tuple) else plan
        if traj and getattr(traj, "joint_trajectory", None) and traj.joint_trajectory.points:
            ok2 = commander.execute(traj, wait=True)
            commander.stop()
            if ok2:
                rospy.loginfo("Sichere Gelenkpose erreicht (Fallback).")
                return True
    except Exception as e:
        rospy.logwarn(f"[SafePose Fallback] {e}")

    rospy.logwarn("Sichere Gelenkpose NICHT erreicht.")
    return False

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

# === Ergänzung: Hilfsfunktionen bei joint limits =====================
def retreat_up(commander, dz=0.05):
    """Versuche, aus der aktuellen Pose 5 cm in Welt-Z nach oben zu fahren."""
    try:
        current = commander.get_current_pose(end_effector_link=commander.get_end_effector_link()).pose
        target = copy.deepcopy(current)
        target.position.z += dz
        plan, frac = commander.compute_cartesian_path([target], eef_step=0.01, avoid_collisions=True, jump_threshold=0.0)
        if frac > 0.8:
            ok = commander.execute(plan, wait=True)
            return bool(ok)
    except Exception as e:
        rospy.logwarn(f"[Recovery] retreat_up failed: {e}")
    return False


def retreat_xy(commander, dx=0.07, dy=0.0):
    """Kartesisch seitlich weg, falls Z blockiert ist."""
    try:
        current = commander.get_current_pose(end_effector_link=commander.get_end_effector_link()).pose
        target = copy.deepcopy(current)
        target.position.x += dx
        target.position.y += dy
        plan, frac = commander.compute_cartesian_path([target], eef_step=EEF_STEP, avoid_collisions=True, jump_threshold=0.0)
        if frac > 0.3:
            ok = commander.execute(plan, wait=True)
            return bool(ok)
    except Exception as e:
        rospy.logwarn(f"[Recovery] retreat_xy failed: {e}")
    return False


def recover_to_safe(commander):
    """
    Stoppen -> mehrere Rettungsversuche (Z, dann XY) -> sichere Pose.
    Liefert True nur, wenn move_to_safe_pose() wirklich geklappt hat.
    """
    try:
        commander.stop()
        rospy.sleep(0.2)
        commander.clear_path_constraints()
        commander.clear_pose_targets()

        # 1) Mehrere Z-Retreats versuchen
        for dz in (0.03, 0.06, 0.10):
            if retreat_up(commander, dz=dz):
                break
        else:
            rospy.logwarn("[Recovery] retreat_up in allen Stufen fehlgeschlagen.")

            # 2) XY-Retreat-Kombis probieren (kleine Schritte, 7 cm)
            for (dx, dy) in ((0.07, 0.0), (-0.07, 0.0), (0.0, 0.07), (0.0, -0.07)):
                if retreat_xy(commander, dx=dx, dy=dy):
                    break
            else:
                rospy.logwarn("[Recovery] retreat_xy ebenfalls fehlgeschlagen.")

        # 3) sichere Pose (liefert True/False)
        ok = move_to_safe_pose(commander)
        if not ok:
            rospy.logwarn("[Recovery] sichere Pose NICHT erreicht.")
        return ok

    except Exception as e:
        rospy.logerr(f"[Recovery] Ausnahme: {e}")
        return False
    
APPROACH_DZ = 0.04    # 4 cm über Ziel anfahren
EEF_STEP    = 0.005
FRACTION_REQ = 0.8

def plan_and_execute_cartesian(commander, waypoints, eef_step=EEF_STEP):
    commander.set_start_state_to_current_state()
    plan, frac = commander.compute_cartesian_path(
        waypoints, eef_step=eef_step, avoid_collisions=True, jump_threshold=0.0)
    if not plan or not getattr(plan, "joint_trajectory", None) or not plan.joint_trajectory.points:
        return False, frac
    if frac < FRACTION_REQ:
        return False, frac
    ok = commander.execute(plan, wait=True)
    return bool(ok), frac

def try_approach_and_descend(commander, goal_pose, approach_dz=APPROACH_DZ):
    pre = copy.deepcopy(goal_pose)
    pre.position.z += approach_dz
    ok1, _ = plan_and_execute_cartesian(commander, [pre])
    if not ok1:
        return False
    ok2, _ = plan_and_execute_cartesian(commander, [goal_pose])
    if not ok2:
        return False
    return True

def fallback_pose_goal(commander, pose):
    commander.clear_path_constraints()
    commander.set_start_state_to_current_state()
    commander.set_pose_target(pose)
    plan = commander.plan()
    traj = plan[1] if isinstance(plan, tuple) else plan
    if not traj or not getattr(traj, "joint_trajectory", None) or not traj.joint_trajectory.points:
        commander.clear_pose_targets()
        return False
    ok = commander.execute(traj, wait=True)
    commander.clear_pose_targets()
    return bool(ok)


def raster_scan(ctx, config, marker_pub, box_pose, probe_length=0.1):
    # speicherpfad CSV-Datei
    results_dir = os.path.join(os.path.expanduser("~"), "catkin_ws", "raster_results")
    os.makedirs(results_dir, exist_ok=True)  # <— ergänzen
    # Zeitstempel für DAteinamen
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")  #datetime.
    output_file = os.path.join(results_dir, f"raster_log_{timestamp}.csv")
    # Datei initialisieren/ in Header schreiben
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        #writer.writerow(["x", "y", "z", "tilt_axis", "tilt_angle_deg", "reachable", "fraction", "radial_distance"])
        writer.writerow(["x", "y", "z", "tilt_axis", "tilt_angle_deg", "reachable", "fraction", "radial_distance", "time_start", "time_end"])

    print(f"Schreibe Ergebnis-CSV nach: {output_file}")

    scene = ctx.scene
    commander = ctx.commander

    rospy.sleep(2)

    # Marker
    #marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
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
    
    total = len(heights) * raster_size**2
    idx = 0
    for z in heights:
        for i in range(raster_size):
            for j in range(raster_size):
                idx += 1
                probe_tip = np.array([x_start + i * delta, y_start + j * delta, z]) #target location für die Sondenspitze
                tilt_axis = 'x' if (idx % 2 == 1) else 'y'# rest 1 ungerade, rest 0 gerade

                for tilt_angle in tilt_angles_rad:
                    if tilt_axis == 'x':
                        tilt_quat = quaternion_from_euler(tilt_angle, 0, 0) #rotation um x
                    else:
                        tilt_quat = quaternion_from_euler(0, tilt_angle, 0) #rotation um y
                    quat = quaternion_multiply(tilt_quat, base_quat) #ganze rotation basierend auf tilt und anfangspose/rotation
                    pose_goal = compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length)

                    # conbstraints setzen und planen
                    commander.set_path_constraints(ctx.build_constraints(quat))
                    commander.set_start_state_to_current_state()

                    plan, fraction = commander.compute_cartesian_path(
                        [pose_goal], eef_step=0.01, avoid_collisions=True, jump_threshold = 0.0) #berechne linear interpolierten pfad/trajetory für meine zielpose

                    commander.clear_path_constraints()

                    tilt_deg = np.rad2deg(tilt_angle)
                    if fraction > 0.8:
                        # radialer Abstand (wie gehabt)
                        box_x = box_pose.pose.position.x
                        box_y = box_pose.pose.position.y
                        box_z = box_pose.pose.position.z
                        radial_dist = np.linalg.norm([
                            probe_tip[0] - box_x,
                            probe_tip[1] - box_y,
                            probe_tip[2] - box_z
                        ])

                        start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                        exec_ok = False
                        try:
                            # 1) Cartesian: Pre-Approach -> Ziel
                            exec_ok = try_approach_and_descend(commander, pose_goal, approach_dz=APPROACH_DZ)

                            # 2) Fallback auf normale Pose-Planung
                            if not exec_ok:
                                rospy.logwarn("[Fallback] Cartesian fehlgeschlagen, versuche Pose-Planung...")
                                exec_ok = fallback_pose_goal(commander, pose_goal)

                            if exec_ok:
                                rospy.sleep(t)  # Verweildauer
                            else:
                                raise RuntimeError("alle Ausführungsstrategien fehlgeschlagen")

                        except Exception as e:
                            rospy.logerr(f"[✗] Ausführung fehlgeschlagen bei Rasterpunkt {idx} (Tilt-{tilt_axis} {tilt_deg:.1f}°): {e}")
                            if recover_to_safe(commander):
                                rospy.loginfo("[Recovery] In sichere Pose zurückgekehrt.")
                            else:
                                rospy.logwarn("[Recovery] Konnte nicht sicher zurückkehren – weiter mit nächstem Punkt.")
                        finally:
                            commander.stop()
                            commander.clear_path_constraints()
                            commander.clear_pose_targets()

                        end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S") if exec_ok else ""

                        # Ein Log-Eintrag
                        with open(output_file, mode='a', newline='') as file:
                            writer = csv.writer(file)
                            writer.writerow([
                                probe_tip[0], probe_tip[1], probe_tip[2],
                                tilt_axis, tilt_deg,
                                bool(exec_ok), round(fraction, 2),
                                round(radial_dist, 4),
                                start_time, end_time
                            ])

                        if exec_ok:
                            rospy.loginfo(f"[✓] Rasterpunkt {idx}: x={probe_tip[0]:.3f}, y={probe_tip[1]:.3f}, z={probe_tip[2]:.3f}, Tilt-{tilt_axis}: {tilt_deg:.1f}°")
                            probe_marker.points.append(Point(*probe_tip))
                            marker_pub.publish(probe_marker)
                        else:
                            rospy.logwarn(f"[→] Rasterpunkt {idx} übersprungen (Fehler bei Ausführung).")

    

                    else:
                        # radialer Abstand zur boxmitte berechnen 3D
                        box_x = box_pose.pose.position.x
                        box_y = box_pose.pose.position.y
                        box_z = box_pose.pose.position.z

                        radial_dist = np.linalg.norm([
                            probe_tip[0] - box_x,
                            probe_tip[1] - box_y,
                            probe_tip[2] - box_z
                        ])
                        #radial_dist = np.linalg.norm([probe_tip[0] - box_x, probe_tip[1] - box_y])

                        # Ergebniszeile zur Datei hinzufügen
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
    box_pose = setup_environment(ctx.scene, ctx.commander)

    while not rospy.is_shutdown():
        print("\n=== Raster-Optionen ===")
        print("1: Standardraster (raster_size=5, heights=[0.02, 0.05, 0.1, 0.15, 0.2], tilt_angles=[±20°, ±40°])")
        print("2: Einfaches Test-Raster um OCR zu testen (raster_size=2, heights=[0.02, 0.5], tilt_angles=[0, +20°])")
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
                    "raster_size": 5,
                    "delta": 0.05,
                    "heights": [0.02, 0.05, 0.1, 0.15, 0.2],
                    "tilt_angles_deg": [0, 20, 40, -20, -40]
                }

            if raster_choice == '2':
                config = {
                    "raster_size": 3,
                    "delta": 0.05,
                    "heights": [0.2],
                    "tilt_angles_deg": [0, 20, -20]
                }

            elif raster_choice == '3':
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
    #time.sleep(5)

    ctx = Context('panda_arm') # meine moveit gruppe (moveit setup)
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    raster_menu(ctx)
    