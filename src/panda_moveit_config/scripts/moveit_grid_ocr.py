#!/usr/bin/env python3
import re
import mss
from PIL import Image
import pytesseract

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
#t = 1  #verweildauer der sonde am punkt


pytesseract.pytesseract.tesseract_cmd = os.environ.get("TESSERACT_CMD", "/usr/bin/tesseract")

# #OBS muss oben in die linke ecke plaziert werden, mit der usprungsgröße wenn ich es lffne, dann ist die REGIOn richtig ausgerichtet 
REGION = {"top": 100, "left": 300, "width": 800, "height": 500}


class Context:
    def __init__(self, move_group_name):
        rospy.loginfo(node_prefix + 'Waiting for move_group/status')
        rospy.wait_for_message('move_group/status', actionlib_msgs.msg.GoalStatusArray, timeout)

        self.commander = moveit_commander.MoveGroupCommander(move_group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

        self.commander.set_end_effector_link('panda_sensor_mount') #panda_link8
        self.commander.set_planning_time(30) #30
        self.commander.set_num_planning_attempts(3) #5
        
        #meine go position ist
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
    wall_pose.pose.position.x = -0.5
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
    box_pose.pose.position.x = 0.35  #0.325:  0.3 funktioniert mit der höhe 0.2=z nicht 
    box_pose.pose.position.z = 0.005
    #box
    #smal_box_size = (0.02, 0.02, 0.01)
    #scene.add_box("small_box", box_pose, smal_box_size)
    #kreis
    radius = 0.02 # 2cm
    height_strahler = 0.01
    scene.add_cylinder("small_cyl", box_pose, height_strahler, radius)
    smal_circ_size=(2*radius, 2*radius,height_strahler)

    rospy.sleep(2)
    return box_pose, smal_circ_size

def ensure_results_dir():
    res_dir = os.path.join(os.path.expanduser("~"), "catkin_ws", "raster_results")
    os.makedirs(res_dir, exist_ok=True)
    return res_dir

def capture_ocr_series(duration_s, interval_s, region, out_dir, basename_prefix):
    """
    Nimmt für 'duration_s' jede 'interval_s' Sekunden einen Screenshot.
    Speichert Bilder und schreibt eine CSV mit OCR-Ergebnissen.
    Rückgabe: (csv_path, n_captures)
    """
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"{basename_prefix}_ocr.csv")
    start = time.monotonic()
    next_tick = start
    idx = 0

    with open(csv_path, "w", newline="") as fcsv, mss.mss() as sct:
        w = csv.writer(fcsv)
        w.writerow(["idx", "t_iso", "elapsed_s", "ocr_time", "ocr_date", "ocr_text", "img_path"])

        while True:
            now = time.monotonic()
            if now - start >= duration_s:
                break

            # Screenshot
            sct_img = sct.grab(region)
            img = Image.frombytes("RGB", sct_img.size, sct_img.rgb)

            # Vorverarbeitung
            img_gray = img.convert("L")
            img_bw = img_gray.point(lambda x: 0 if x < 128 else 255, "1")
            img_scaled = img_bw.resize((img_bw.width * 2, img_bw.height * 2))

            # OCR
            text = pytesseract.image_to_string(
                img_scaled, config="--psm 6 -c tessedit_char_whitelist=0123456789:."
            ).strip()
            m_time = re.search(r"\d{1,2}:\d{2}", text)
            m_date = re.search(r"\d{1,2}\.\d{1,2}\.\d{4}", text)
            ocr_time = m_time.group(0) if m_time else ""
            ocr_date = m_date.group(0) if m_date else ""

            # speichern
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            img_path = os.path.join(out_dir, f"{basename_prefix}_{ts}_{idx:03d}.png")
            img_scaled.save(img_path)

            idx += 1
            w.writerow([idx, datetime.now().isoformat(), round(now - start, 3), ocr_time, ocr_date, text, img_path])

            # präzises Intervall
            next_tick += interval_s
            sleep_for = max(0.0, next_tick - time.monotonic())
            time.sleep(sleep_for)

    return csv_path, idx


def raster_scan(ctx, config, marker_pub, box_pose, box_size, probe_length=0.1):

    scene = ctx.scene
    commander = ctx.commander

    results_dir = ensure_results_dir()

    screenshots_root = os.path.join(results_dir, "screenshots")  

    run_ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    basename = f"raster_log_{run_ts}"
    summary_path = os.path.join(results_dir, f"{basename}_summary.csv")

    # Summary-CSV vorbereiten (einmal pro Run)
    if not os.path.exists(summary_path):
        with open(summary_path, "w", newline="") as fsum:
            w = csv.writer(fsum)
            w.writerow(["z", "attempt", "points_total", "points_reached", "reach_fraction",
                         "delta", "raster_size", "auto_success", "started_at", "finished_at", "csv_path"])

    # Marker für RViz
    probe_marker = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD)
    probe_marker.scale.x = probe_marker.scale.y = probe_marker.scale.z = 0.02
    probe_marker.color.b = 1.0
    probe_marker.color.a = 1.0
    probe_marker.header.frame_id = "panda_link0"
    probe_marker.points = []

    # Keine Winkel mehr -> Basis-Orientierung „nach unten“
    base_quat = quaternion_from_euler(-np.pi / 2, 0, 0)

    raster_size = int(config["raster_size"])
    delta = float(config["delta"])
    heights = list(config["heights"])
    edge_clearance = float(config.get("edge_clearance", 0.01))  # Abstand zur Boxkante
    measurement_time_seconds = int(config.get("dwell_seconds", 60))
    # in raster_scan(), direkt nach dem Einlesen der config:
    dwell_seconds = int(config.get("dwell_seconds", config.get("measurement_time", 60)))


    # Grid-Breite
    width = (raster_size - 1) * delta

    # Box-Zentrum/Kanten
    bx, by, bz = box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z
    half_x = box_size[0] / 2.0

    # y mittig um Box
    y_start_centered = by - ((raster_size - 1) / 2.0) * delta

    # x nur auf EINER Seite der Box — fest: RIGHT
    #side = "right"
    x_start = bx + half_x + edge_clearance

    rospy.sleep(2)

    # --- Hauptschleife über Höhen ---
    for z in heights:
        attempt = 0
        while True:
            attempt += 1
            height_started = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # Per-Höhe CSV (ggf. mit _retryN)
            z_mm = int(round(z * 1000))
            height_tag = f"z{z_mm:.3f}mm".replace('.', '_')
            retry_tag = "" if attempt == 1 else f"_retry{attempt-1}"
            csv_path = os.path.join(results_dir, f"{basename}_{height_tag}{retry_tag}.csv")
            with open(csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "x","y","z","reachable","fraction","radial_distance",
                    "time_start","time_end","ocr_series_csv","n_captures"
                ])

            points_total = 0
            points_reached = 0

            for i in range(raster_size):
                for j in range(raster_size):
                    points_total += 1

                    probe_tip = np.array([
                        x_start + i * delta,
                        y_start_centered + j * delta,
                        z
                    ])

                    quat = base_quat  # keine Winkel mehr
                    pose_goal = compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length)

                    # planen (Kartesisch)
                    commander.set_path_constraints(ctx.build_constraints(quat))
                    commander.set_start_state_to_current_state()
                    plan, fraction = commander.compute_cartesian_path(
                        [pose_goal],
                        eef_step=0.005,
                        avoid_collisions=True,
                        jump_threshold=0.0
                    )
                    commander.clear_path_constraints()

                    # Distanz zum Strahlerzentrum (für Log)
                    radial_dist = np.linalg.norm([
                        probe_tip[0] - bx,
                        probe_tip[1] - by,
                        probe_tip[2] - bz
                    ])

                    if fraction > 0.8:
                        start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        commander.execute(plan, wait=True)
                        #rospy.sleep(t)

                        point_dir = os.path.join(screenshots_root, height_tag, f"pt_i{i:02d}_j{j:02d}")
                        series_prefix = f"{basename}_{height_tag}_i{i:02d}_j{j:02d}"
                        try:
                            ocr_csv_path, n_caps = capture_ocr_series(
                                duration_s=dwell_seconds, interval_s=1.0,
                                region=REGION, out_dir=point_dir, basename_prefix=series_prefix
                            )
                        except Exception as e:
                            rospy.logwarn(f"[OCR] Fehler bei Serie: {e}")
                            ocr_csv_path, n_caps = "", 0

                        end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        points_reached += 1

                        with open(csv_path, "a", newline="") as f:
                            w = csv.writer(f)
                            w.writerow([
                                probe_tip[0], probe_tip[1], probe_tip[2],
                                True, round(fraction, 2), round(radial_dist, 5),
                                start_time, end_time, ocr_csv_path, n_caps
                            ])

                        probe_marker.points.append(Point(*probe_tip))
                        marker_pub.publish(probe_marker)
                        rospy.loginfo(f"[✓] {height_tag} pt: x={probe_tip[0]:.3f}, y={probe_tip[1]:.3f}, z={probe_tip[2]:.3f} (fraction={fraction:.2f})")

                    else:
                        with open(csv_path, "a", newline="") as f:
                            w = csv.writer(f)
                            w.writerow([
                                probe_tip[0], probe_tip[1], probe_tip[2],
                                False, round(fraction, 2), round(radial_dist, 5),
                                "", "", "", 0
                            ])
                        rospy.logwarn(f"[✗] {height_tag} NICHT erreichbar (fraction={fraction:.2f})")

            # --- Höhe abgeschlossen: Auto-Erfolg + Summary-Log + einzige Eingabe ---
            height_finished = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            reach_fraction = (points_reached / float(points_total)) if points_total else 0.0
            auto_success = (points_reached == points_total)

            print("\n--- Höhen-Zwischenbilanz ---")
            print(f"Höhe z={z:.3f} m")
            print(f"Erreicht: {points_reached}/{points_total}  (fraction={reach_fraction:.2%})")
            if auto_success:
                print("Ergebnis: ERFOLGREICH (alle Punkte erreicht)")
            else:
                print("Ergebnis: NICHT vollständig (einige Punkte nicht erreicht)")

            with open(summary_path, "a", newline="") as fsum:
                w = csv.writer(fsum)
                w.writerow([z, attempt, points_total, points_reached, round(reach_fraction, 4),
                             delta, raster_size, auto_success, height_started, height_finished, csv_path])

            rospy.loginfo(f"[INFO] CSV für z={z:.3f} m gespeichert: {csv_path}")

            # Optionen: [j] wiederholen, [n] weiter, [a] abbrechen
            while True:
                ans = input("Aktion? [j] wiederholen / [n] weiter / [a] abbrechen: ").strip().lower()
                if ans in ('j', 'y', 'ja'):
                    # gleiche Höhe erneut fahren (neue CSV mit _retryN)
                    break_loop = False
                    break  # zurück zur while-True-Schleife -> attempt+1
                elif ans in ('n', 'w', 'weiter'):
                    # nächste Höhe
                    break_loop = True
                    break
                elif ans in ('a', 'abbrechen', 'q'):
                    rospy.loginfo("Vom Nutzer abgebrochen. Fahre in sichere Pose …")
                    move_to_safe_pose(commander)
                    return  # ganze Funktion verlassen
                else:
                    print("Ungültige Eingabe. Bitte j/n/a.")


            if break_loop:
                # raus aus der while-True-Schleife -> weiter zur nächsten z-Höhe
                break
            # sonst: wiederholen (while True läuft weiter, attempt wird erhöht)

    rospy.loginfo("Fahre in sichere Pose zurück...")
    move_to_safe_pose(commander)


def raster_menu(ctx):
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    box_pose, box_size = setup_environment(ctx.scene, ctx.commander)

    while not rospy.is_shutdown():
        print("\n=== Raster-Optionen ===")
        print("1: Raster (nur eine Seite der Box) – standard")
        print("q: Beenden")
        choice = input("Wähle (1/q): ").strip().lower()
        if choice == 'q':
            print("Beende Raster-Demo.")
            break
        if choice != '1':
            print("Ungültige Eingabe.")
            continue

        config = {
            "raster_size": 5,
            "delta": 0.002,
            "heights": [0.2, 0.201],  # Beispiele
            "edge_clearance": 0.01,  # 1 cm Abstand von der Boxkante
            # keine tilt angles mehr
            "measurement_time": 60
        }

        raster_scan(ctx, config, marker_pub, box_pose, box_size)

        again = input("\nNochmal ausführen (j/n)? ").strip().lower()
        if again != 'j':
            break


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('raster_scan_demo')
    ctx = Context('panda_arm')
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    raster_menu(ctx)