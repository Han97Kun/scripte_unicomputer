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

import re
import mss
from PIL import Image
import pytesseract

from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint, DisplayTrajectory
from actionlib_msgs.msg import GoalStatusArray


#eingie links: https://projects.saifsidhik.page/franka_ros_interface/DOC.html#module-franka_moveit
#eingie links: https://projects.saifsidhik.page/franka_ros_interface/DOC.html#module-franka_moveit

#TODO
    # im script soll dann der ortsverkotr der sonde mit angegegben werden, da in dieser Art deer winkel eingegebe wird 
    # x position relativ angeben !!
    # ersten 4 bilder mindesten löschen, da diese unegnau sind durch den positionswechserl 
    # 
    # calibrierung nuklid richtige !!
    # höhe und radius der gammaquelle ausmessen und anpassen für die linalg berechnung (nimmt er oberfläche oder was nimmt er für linalg )
    #


'''source devel/setup.bash
cd ~/catkin_ws/

rosrun panda_moveit_config final_rasterscan.py

rosrun results_sonde match_cps_to_position.py

roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=172.16.0.2
roslaunch panda_moveit_config demo.launch

'''



'''
######github pushen

cd ~/catkin_ws

# sicherstellen, dass kein Rebase mehr läuft
git rebase --abort 2>/dev/null || true

# neuen Branch erstellen
git checkout -b catkin_ws_new

# alles hinzufügen
git add .
git commit -m "Upload catkin_ws as catkin_ws_new branch"

# pushen
git push -u origin catkin_ws_new


'''


# ===================================
# Global Variables
# ===================================
'''
timeout         = time in seconds to wait for move_group/status
t_0             = time in seconds to align the gamma source (only for simulation needed)
t_1             = time in seconds for countingt the cps of the gamma source for each position (simulation: t_1 = 2, real measurements = t_1=60)
screen_region   = region of the screen to capture (OBS needs to be placed in the upper left corner of the screen with the original size when opening it, then the region is correctly aligned. Scale the OBS window to 100%)
'''
node_prefix = 'raster_scan_demo: '
timeout     = 10.0
t_0         = 4 
t_1         = 3

# Optical CHaracter Recognition (OCR) for reading the CPS from the screen
pytesseract.pytesseract.tesseract_cmd = os.environ.get("TESSERACT_CMD", "/usr/bin/tesseract")
screen_region = {"top": 100, "left": 400, "width": 300, "height": 150} # full screen und maximale resolution 2048x1080
#screen_region = {"top": 110, "left": 180, "width": 100, "height": 40} #640zu360 kleinste resolution in obs

# ===================================
# Class and Function Definitions
    # Context class
    # move_to_safe_pose
    # compute_effector_pose_for_probe_tip
    # setup_environment
# ===================================

class Context:
    '''
    Context: Class for MoveIt Commander and PlanningSceneInterface
        MoveIt Commander: maximal velocity and acceleration scaling factors are set here as well as planning time and number of planning attempts
        PlanningSceneInterface: used to add collision objects to the planning scene
        build_constraints: builds the constraints for the motion planning (not used in this script)
    '''
    def __init__(self, move_group_name):
        rospy.loginfo(node_prefix + 'Waiting for move_group/status')
        rospy.wait_for_message('move_group/status', actionlib_msgs.msg.GoalStatusArray, timeout) # wait for the action server to be ready

        self.commander = moveit_commander.MoveGroupCommander(move_group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

        self.commander.set_end_effector_link('panda_sensor_mount') # panda_link8
        self.commander.set_planning_time(30) 
        self.commander.set_num_planning_attempts(3) 
        
        self.commander.set_max_velocity_scaling_factor(0.05) # 5% of maximum velocity
        self.commander.set_max_acceleration_scaling_factor(0.03) # 3% of maximum acceleration
        
    def build_constraints(self, quat, joint7_angle=np.pi / 4):
        constraints = Constraints()

        # PositionConstraint (nur Y >= 0.0 zulÃ¤ssig)
        pc = PositionConstraint()
        pc.header.frame_id = "panda_link0"
        pc.link_name = "panda_sensor_mount"
        pc.weight = 1.0

        # Definiere ein Box-Primitiv, das nur Y >= 0 erlaubt
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [3.0, 3.0, 3.0]  # [x, y, z] GrÃ¶ÃŸe

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
    """
    This function moves the robot to a predefined and safe position
    """

    joint_goal = commander.get_current_joint_values() # curretn joint values

    # set safe joint values (tested in Rviz)
    joint_goal[0] = 0.0                  # panda_joint1 = 0Â°
    joint_goal[1] = -np.pi / 4           # panda_joint2 = -45Â°
    joint_goal[2] = 0.0                  # panda_joint3 = 0Â° 
    joint_goal[3] = -np.pi*3 / 4         # 135Â°
    joint_goal[4] = 0.0                  # 0Â°        
    joint_goal[5] = np.pi / 2            # 90Â°
    joint_goal[6] = np.pi / 4            # 45Â°

    rospy.loginfo("Move to safe position ...")
    commander.go(joint_goal, wait=True) # moves the robot to the safe position
    commander.stop()
    rospy.loginfo("Reached safe position.")
 
def compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length=0.1):
    """
    This function computes the pose of the endeffector based on the target position for the probe_tip.
    probe_lenght: distance between endeffector and probe tip in m
    probe_tip: target position of the probe tip (distal end of the probe) in world coordinates (m)
    effector_pose: pose of the endeffector in world coordinates (m)
    """
    rot_matrix = quaternion_matrix(quat)[:3, :3] # orientation of the endeffector in world coordinates
    offset_world = rot_matrix @ np.array([0, probe_length, 0]) # Vector from endeffector to probe tip (coordinate system of the porbe tip is different to the world coordinate system)
    effector_position = probe_tip - offset_world # prob_tip (target position of the probe tip) to calculate the endeffector position

    pose = Pose() 
    pose.position.x, pose.position.y, pose.position.z = effector_position # set position of the endeffector
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat # set orientation of the endeffector

    return pose

def setup_environment(scene, commander):
    """
    This function sets up the environment in Rviz.
    It adds a table, a wall, four legs, and the gamma source itself and the Lagerung of the source(sponge) to the scene to avoide collisions.
    """
    rospy.sleep(2)
    
    # Table
    table_pose = PoseStamped()
    table_pose.header.frame_id = commander.get_planning_frame()
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.x = 0.3
    table_pose.pose.position.z = -0.0251
    scene.add_box("table", table_pose, (0.9, 0.9, 0.05))

    # Wall
    wall_pose = PoseStamped()
    wall_pose.header.frame_id = commander.get_planning_frame()
    wall_pose.pose.orientation.w = 1.0
    wall_pose.pose.position.x = -0.5
    wall_pose.pose.position.z = 0.5
    scene.add_box("wall", wall_pose, (0.005, 2.0, 2.0))

    # 4 Legs for the table
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

    # Gamma source (including the sponge height)
    source_pose = PoseStamped()
    source_pose.header.frame_id = commander.get_planning_frame()
    source_pose.pose.orientation.w = 1.0
    source_pose.pose.position.x = 0.35 

    # 1.6 cm ist die plstiv bverpcakung hoch von strahlequelle (zusammenb 4.4)
    # 2.1 cm hoch ist dieBlei kugel box
    # 2.3 cm sind die beiden Aluminieum blöcke
    # 4mm dickes blei
    # 1.1cm ist die große wasser box
    # 4mm ist die quelle selber
    #3cm ist der schwamm hoch
    # sizes of the gamma source and the sponge (just height of the sponge 0.03)
    height_sponge          = 0.012 # 0.03 +height of the sponge in meters (2.9cm)  #1.4cm ist die mitgelieferte box
    radius                 = 0.01 # Radius of the Gamma source in m (1cm radius, diameter 2cm)
    height_strahler        = 0.005 # height of the gamma source in meters (4mm)
    height_strahler_sponge = height_strahler+height_sponge # total height of the gamma source including the sponge

    base_z = 0.0 # base of the gamma source (without sponge) is at z=0.0m (table top)
    source_pose.pose.position.z = base_z + height_strahler_sponge/2 # MoveIt places the box center at the given position, so we need to add half of the height of the gamma source including the sponge

    scene.add_cylinder("small_cyl", source_pose, height_strahler_sponge, radius)
    source_size=(2*radius, 2*radius,height_strahler_sponge)

    rospy.sleep(2)
    return source_pose, source_size

def ensure_results_dir():
    res_dir = os.path.join(os.path.expanduser("~"), "catkin_ws", "raster_results")
    os.makedirs(res_dir, exist_ok=True)
    return res_dir

def source_alignment(ctx, source_pose, source_size, probe_length=0.1,
                     hover_over_surface=0.02):

    commander = ctx.commander
    base_quat = quaternion_from_euler(-np.pi/2, 0, 0)  # orientation of the probe (pointing downwards)

    bx, by = source_pose.pose.position.x, source_pose.pose.position.y
    z_surface_top = source_size[2] #bz

    z_alignment = z_surface_top + 0.01 # hight for source alignemnt, 1cm above the sponge and the source
    tip_alignment = np.array([bx, by, z_alignment])  # center of the source

    rospy.loginfo(f"z_alignemnt={z_alignment:.3f},source_size={source_size[2]:.3f}")

    pose_over_cyl = compute_effector_pose_for_probe_tip(tip_alignment, base_quat, probe_length)

    commander.set_start_state_to_current_state()
    plan_over, frac_over = commander.compute_cartesian_path(
        [pose_over_cyl], eef_step=0.005, avoid_collisions=True, jump_threshold=0.0
    )
    commander.execute(plan_over, wait=True)
    rospy.sleep(t_0)  
    rospy.loginfo(f"Approach: z={z_alignment:.3f} m")
    rospy.loginfo(f"Stop the robot to align the gamma source for.")

def capture_ocr_series(duration_s, interval_s, region, out_dir, basename_prefix):
    """
    Nimmt für 'duration_s' jede 'interval_s' Sekunden einen Screenshot.
    Speichert Bilder (nach Vorverarbeitung) und schreibt eine CSV mit OCR-Ergebnissen.
    rückgane: (csv_path, n_captures)
    """
    import csv, time, re
    import mss
    from PIL import Image, ImageOps, ImageFilter
    import pytesseract
    from datetime import datetime
    import os

    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"{basename_prefix}_ocr.csv")
    start = time.monotonic()
    next_tick = start
    idx = 0

    # Tesseract-Config: nur Ziffern, psm 7 (einzelne Zeile), oem 3 (LSTM)
    tess_cfg = r"--oem 1 --psm 8 -c tessedit_char_whitelist=0123456789o"

    with open(csv_path, "w", newline="") as fcsv, mss.mss() as sct:
        w = csv.writer(fcsv)
        w.writerow(["idx", "t_iso", "elapsed_s", "ocr_text", "extracted_number", "img_path"])

        while True:
            now = time.monotonic()
            if now - start >= duration_s:
                break

            # Screenshot
            sct_img = sct.grab({
                "top": int(region["top"]),
                "left": int(region["left"]),
                "width": int(region["width"]),
                "height": int(region["height"]),
            })
            img = Image.frombytes("RGB", sct_img.size, sct_img.rgb)

            # ---- Vorverarbeitung  ----
            # 1) Graustufen
            g = img.convert("L")
            # 3) Invertieren (weiß auf schwarz -> schwarz auf weiß)
            g = ImageOps.invert(g)
            # 4) Autokontrast
            g = ImageOps.autocontrast(g, cutoff=1)

            # OCR (nur Ziffern)
            text_raw = pytesseract.image_to_string(g, lang="eng", config=tess_cfg).strip()
            text_fixed = text_raw.replace("o", "0")#.replace("o", "0")

            digits = "".join(ch for ch in text_fixed if ch.isdigit())
            extracted_number = int(digits) if digits else None

            # speichern (verarbeitete Version â€“ das ist das Bild, das Tesseract sieht)
            ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S.%f")[:-3]
            img_path = os.path.join(out_dir, f"{basename_prefix}_{ts}_{idx:03d}.png")
            g.save(img_path)

            idx += 1
            print(f"[OCR] {digits if digits else text_fixed}")

            w.writerow([
                idx,
                datetime.now().isoformat(),
                round(now - start, 3),
                text_raw,
                extracted_number,
                img_path
            ])

            #  Intervall
            next_tick += interval_s
            time.sleep(max(0.0, next_tick - time.monotonic()))

    return csv_path, idx


 # --- Helper: Ordnername aus XY bauen ---

def xy_dirname(probe_tip, unit="mm"):
    x, y = float(probe_tip[0]), float(probe_tip[1])
    if unit == "mm":
        xi = int(round(x * 1000.0))
        yi = int(round(y * 1000.0))
        return f"x{xi}mm_y{yi}mm"
    xs = f"{x:.3f}".replace('.', '_')
    ys = f"{y:.3f}".replace('.', '_')
    return f"x{xs}m_y{ys}m"

def raster_scan_2(ctx, config, marker_pub, source_pose, source_size, probe_length=0.1):
    """
    Line scan.
    Config: 'heights' sind RELATIVE Höhen (m) über der Oberkante der Quelle.
    Gefahren wird z_abs = z_surface_top + z_rel.
    CSV loggt z_abs und z_rel_surface_m; radial_distance ist relativ zur Oberkante.
    """
    scene = ctx.scene
    commander = ctx.commander

    results_dir = ensure_results_dir()
    screenshots_root = os.path.join(results_dir, "screenshots")

    run_ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    basename = f"raster_log_{run_ts}"

    # Marker in RViz
    probe_marker = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD)
    probe_marker.scale.x = probe_marker.scale.y = probe_marker.scale.z = 0.02
    probe_marker.color.b = 1.0
    probe_marker.color.a = 1.0
    probe_marker.header.frame_id = "panda_link0"
    probe_marker.points = []

    # Endeffektor zeigt nach -Z
    base_quat = quaternion_from_euler(-np.pi / 2, 0, 0)

    # Konfigurationsparameter
    raster_size = int(config["raster_size"])
    delta = float(config["delta"])  # Schrittweite kommt aus Config
    heights_rel = list(config["heights"])

    # OCR-Parameter
    measurement_time_seconds = int(config.get("measurement_time_seconds", t_1))
    ocr_interval  = float(config.get("ocr_interval_s", 0.25)) 
    enable_ocr    = bool(config.get("enable_ocr", True))
    ocr_region    = config.get("ocr_region", screen_region)

    # Quelle
    bx, by, bz = (source_pose.pose.position.x,
                  source_pose.pose.position.y,
                  source_pose.pose.position.z)
    z_surface_top = source_size[2]
    rospy.sleep(2)

    # Hauptschleife über Höhen
    for z_rel in heights_rel:
        attempt = 0
        while True:
            attempt += 1
            height_started = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            z_mm = int(round(z_rel * 1000.0))
            height_tag = f"z{z_mm}mm"
            csv_tag = f"_{attempt}"
            csv_path = os.path.join(results_dir, f"{basename}_{height_tag}{csv_tag}.csv")
            with open(csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "x","y","z","reachable","fraction","radial_distance",
                    "time_start","time_end","ocr_series_csv","n_captures","ocr_mean",'ocr_variance'
                ])

            points_total = 0
            points_reached = 0

            # ===============================
            # LINIEN-SCAN (x fix, y entlang in delta-Schritten)
            # ===============================
            ys = by + np.arange(raster_size, dtype=float) * delta

            for y_val in ys:
                points_total += 1
                z_abs = z_surface_top + z_rel
                probe_tip = np.array([bx, y_val, z_abs])

                quat = base_quat
                pose_goal = compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length)

                # Trajektorie planen
                commander.set_path_constraints(ctx.build_constraints(quat))
                commander.set_start_state_to_current_state()
                plan, fraction = commander.compute_cartesian_path(
                    [pose_goal],
                    eef_step=0.005,
                    avoid_collisions=True,
                    jump_threshold=0.0
                )
                commander.clear_path_constraints()

                # Abstand relativ zur Oberkante der Quelle
                radial_dist = np.linalg.norm([
                    probe_tip[0] - bx,
                    probe_tip[1] - by,
                    probe_tip[2] - z_surface_top
                ])

                if fraction > 0.8:
                    start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    commander.execute(plan, wait=True)

                    dir_name = f"{height_tag}_{attempt}"
                    point_dir = os.path.join(screenshots_root, dir_name)
                    os.makedirs(point_dir, exist_ok=True)
                    series_prefix = f"{basename}_{height_tag}_{dir_name}"

                    ocr_csv_path, n_caps = "", 0
                    if enable_ocr:
                        try:
                            ocr_csv_path, n_caps = capture_ocr_series(
                                duration_s=measurement_time_seconds,
                                interval_s=ocr_interval,
                                region=ocr_region,
                                out_dir=point_dir,
                                basename_prefix=series_prefix
                            )
                        except Exception as e:
                            rospy.logwarn(f"[OCR] Fehler bei Serie: {e}")

                    end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    points_reached += 1

                    with open(csv_path, "a", newline="") as f:
                        w = csv.writer(f)
                        w.writerow([
                            probe_tip[0], probe_tip[1], z_rel,
                            True, round(fraction, 2), round(radial_dist, 5),
                            start_time, end_time, ocr_csv_path, n_caps
                        ])

                    probe_marker.points.append(Point(*probe_tip))
                    marker_pub.publish(probe_marker)
                    rospy.loginfo(f"[✓] {height_tag} pt: x={probe_tip[0]:.3f}, y={probe_tip[1]:.3f}, "
                                  f"z_rel={z_rel:.3f} (z_abs={probe_tip[2]:.3f}, frac={fraction:.2f})")
                else:
                    with open(csv_path, "a", newline="") as f:
                        w = csv.writer(f)
                        w.writerow([
                            probe_tip[0], probe_tip[1], z_rel,
                            False, round(fraction, 2), round(radial_dist, 5),
                            "", "", "", 0
                        ])
                    rospy.logwarn(f"[✗] {height_tag} NICHT erreichbar (fraction={fraction:.2f})")

            auto_success = (points_reached == points_total)

            print("\n--- height z ---")
            print(f"Height z_rel={z_rel:.3f} m (z_abs={z_surface_top + z_rel:.3f} m)")
            print(f"Reached: {points_reached}/{points_total}")
            print("Result:", "Reached all points successfully" if auto_success else "Not all points reached successfully")

            rospy.loginfo(f"[INFO] CSV for z_rel={z_rel:.3f} m saved: {csv_path}")

            # Optionen: Wiederholen/Weiter/Abbrechen
            while True:
                ans = input("Action? [y] Repeat / [n] Next height / [c] Cancel: ").strip().lower()
                if ans in ('j','y','ja','yes'):
                    break_loop = False
                    break
                elif ans in ('n','w','weiter','next'):
                    break_loop = True
                    break
                elif ans in ('a','abbrechen','q','c'):
                    rospy.loginfo("Canceling raster scan, move to safe pose.")
                    move_to_safe_pose(commander)
                    return
                else:
                    print("Invalid input. [y]=repeat, [n]=next, [c]=cancel.")

            if break_loop:
                break  # nächste Höhe

    rospy.loginfo("Move to safe position.")
    move_to_safe_pose(commander)

def raster_scan(ctx, config, marker_pub, source_pose, source_size, probe_length=0.1):
    """
    Line scan.
    'heights' sind RELATIVE Höhen (m) über der Oberkante der Quelle.
    Gefahren wird z_abs = z_surface_top + z_rel.
    CSV loggt z_rel; radial_distance ist relativ zur Oberkante.
    """
    scene = ctx.scene
    commander = ctx.commander

    results_dir = ensure_results_dir()
    screenshots_root = os.path.join(results_dir, "screenshots")

    run_ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    basename = f"raster_log_{run_ts}"

    # RViz Marker
    probe_marker = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD)
    probe_marker.scale.x = probe_marker.scale.y = probe_marker.scale.z = 0.02
    probe_marker.color.b = 1.0
    probe_marker.color.a = 1.0
    probe_marker.header.frame_id = "panda_link0"
    probe_marker.points = []

    # Endeffektor zeigt -Z
    base_quat = quaternion_from_euler(-np.pi / 2, 0, 0)

    # Konfig
    raster_size = int(config["raster_size"])
    delta       = float(config["delta"])
    heights_rel = list(config["heights"])

    # OCR
    measurement_time_seconds = int(config.get("measurement_time_seconds", t_1))
    ocr_interval  = float(config.get("ocr_interval_s", 0.25))
    enable_ocr    = bool(config.get("enable_ocr", True))
    ocr_region    = config.get("ocr_region", screen_region)

    # Quelle (x fest = bx; y wird hochgezählt)
    bx, by, bz     = (source_pose.pose.position.x,
                      source_pose.pose.position.y,
                      source_pose.pose.position.z)
    z_surface_top  = source_size[2]
    rospy.sleep(2)

    # kleine Hilfe: Ordnername "x350mm_y1mm"
    def xy_dirname_from_xy(x: float, y: float, unit="mm") -> str:
        if unit == "mm":
            xi = int(round(float(x) * 1000.0))
            yi = int(round(float(y) * 1000.0))
            return f"x{xi}mm_y{yi}mm"
        xs = f"{float(x):.3f}".replace('.', '_')
        ys = f"{float(y):.3f}".replace('.', '_')
        return f"x{xs}m_y{ys}m"

    for z_rel in heights_rel:
        attempt = 0
        while True:
            attempt += 1
            height_started = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            z_mm      = int(round(z_rel * 1000.0))
            height_tag = f"z{z_mm}mm"
            csv_tag    = f"_{attempt}"
            csv_path   = os.path.join(results_dir, f"{basename}_{height_tag}{csv_tag}.csv")
            with open(csv_path, "w", newline="") as f:
                csv.writer(f).writerow([
                    "x","y","z","reachable","fraction","radial_distance",
                    "time_start","time_end","ocr_series_csv","n_captures"
                ])

            points_total = 0
            points_reached = 0

            # y-Punkte entlang der Linie: y = by + k*delta
            ys = by + np.arange(raster_size, dtype=float) * delta

            # --- NEU: Höhe-Ordner einmal je Höhe anlegen ---
            height_dir = os.path.join(screenshots_root, height_tag)
            os.makedirs(height_dir, exist_ok=True)

            for y_val in ys:
                points_total += 1
                z_abs = z_surface_top + z_rel
                probe_tip = np.array([bx, y_val, z_abs])

                quat = base_quat
                pose_goal = compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length)

                commander.set_path_constraints(ctx.build_constraints(quat))
                commander.set_start_state_to_current_state()
                plan, fraction = commander.compute_cartesian_path(
                    [pose_goal],
                    eef_step=0.005, avoid_collisions=True, jump_threshold=0.0
                )
                commander.clear_path_constraints()

                radial_dist = np.linalg.norm([
                    probe_tip[0] - bx,
                    probe_tip[1] - by,
                    probe_tip[2] - z_surface_top
                ])

                if fraction > 0.8:
                    start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    commander.execute(plan, wait=True)

                    # --- NEU: Punkt-Unterordner "x350mm_y1mm" unter z{mm}mm ---
                    point_dir_name = xy_dirname_from_xy(probe_tip[0], probe_tip[1], unit="mm")
                    point_dir = os.path.join(height_dir, point_dir_name)
                    os.makedirs(point_dir, exist_ok=True)

                    # --- NEU: Serien-Präfix enthält Höhe + Punktordner ---
                    series_prefix = f"{basename}_{height_tag}_{point_dir_name}"

                    ocr_csv_path, n_caps = "", 0
                    if enable_ocr:
                        try:
                            ocr_csv_path, n_caps = capture_ocr_series(
                                duration_s=measurement_time_seconds,
                                interval_s=ocr_interval,
                                region=ocr_region,
                                out_dir=point_dir,
                                basename_prefix=series_prefix
                            )
                        except Exception as e:
                            rospy.logwarn(f"[OCR] Fehler bei Serie: {e}")

                    end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    points_reached += 1

                    with open(csv_path, "a", newline="") as f:
                        csv.writer(f).writerow([
                            probe_tip[0], probe_tip[1], z_rel,
                            True, round(fraction, 2), round(radial_dist, 5),
                            start_time, end_time,
                            os.path.abspath(ocr_csv_path) if ocr_csv_path else "", n_caps
                        ])

                    probe_marker.points.append(Point(*probe_tip))
                    marker_pub.publish(probe_marker)
                    rospy.loginfo(
                        f"[✓] {height_tag} pt: x={probe_tip[0]:.3f}, y={probe_tip[1]:.3f}, "
                        f"z_rel={z_rel:.3f} (z_abs={probe_tip[2]:.3f}, frac={fraction:.2f})"
                    )
                else:
                    with open(csv_path, "a", newline="") as f:
                        csv.writer(f).writerow([
                            probe_tip[0], probe_tip[1], z_rel,
                            False, round(fraction, 2), round(radial_dist, 5),
                            "", "", "", 0
                        ])
                    rospy.logwarn(f"[✗] {height_tag} NICHT erreichbar (fraction={fraction:.2f})")

            auto_success = (points_reached == points_total)
            print("\n--- height z ---")
            print(f"Height z_rel={z_rel:.3f} m (z_abs={z_surface_top + z_rel:.3f} m)")
            print(f"Reached: {points_reached}/{points_total}")
            print("Result:", "Reached all points successfully" if auto_success else "Not all points reached successfully")
            rospy.loginfo(f"[INFO] CSV for z_rel={z_rel:.3f} m saved: {csv_path}")

            while True:
                ans = input("Action? [y] Repeat / [n] Next height / [c] Cancel: ").strip().lower()
                if ans in ('j','y','ja','yes'):
                    break_loop = False; break
                elif ans in ('n','w','weiter','next'):
                    break_loop = True;  break
                elif ans in ('a','abbrechen','q','c'):
                    rospy.loginfo("Canceling raster scan, move to safe pose.")
                    move_to_safe_pose(commander); return
                else:
                    print("Invalid input. [y]=repeat, [n]=next, [c]=cancel.")
            if break_loop:
                break

    rospy.loginfo("Move to safe position.")
    move_to_safe_pose(commander)

def raster_scan_1(ctx, config, marker_pub, source_pose, source_size, probe_length=0.1):
    """
    Grid scan.
    Config: 'heights' sind RELATIVE Höhen (m) über der Oberkante der Quelle.
    Gefahren wird z_abs = z_surface_top + z_rel.
    CSV loggt z_abs und z_rel_surface_m; radial_distance ist relativ zur Oberkante.
    """
    scene = ctx.scene
    commander = ctx.commander

    results_dir = ensure_results_dir()

    screenshots_root = os.path.join(results_dir, "screenshots")  

    run_ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]## bessere auflÃ¶sung ( in ms) 
    basename = f"raster_log_{run_ts}"


    # Marker in RViz to visualize the raster points
    probe_marker = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD)
    probe_marker.scale.x = probe_marker.scale.y = probe_marker.scale.z = 0.02
    probe_marker.color.b = 1.0
    probe_marker.color.a = 1.0
    probe_marker.header.frame_id = "panda_link0"
    probe_marker.points = []

    # base orientation in -z direction 
    base_quat = quaternion_from_euler(-np.pi / 2, 0, 0)

    # Konfiguration parameter
    raster_size = int(config["raster_size"])
    delta = float(config["delta"])
    heights_rel = list(config["heights"])

    width = (raster_size - 1) * delta

    # Messdauer/Intervalle fÃ¼r OCR
    measurement_time_seconds = int(config.get("measurement_time_seconds", t_1)) #60
    # in raster_scan(), direkt nach dem Einlesen der config:
    ocr_interval  = float(config.get("ocr_interval_s", 0.25)) #0.24=4Hz
    enable_ocr    = bool(config.get("enable_ocr", True))
    ocr_region    = config.get("ocr_region", screen_region)  # erlaubt Override Ã¼ber config

    # center of gamma source
    bx, by, bz = source_pose.pose.position.x, source_pose.pose.position.y, source_pose.pose.position.z
    # startpoint: upper left corner
    x_start = bx 
    y_start = by 
    z_surface_top = source_size[2]
    rospy.sleep(2)

    # main loof do generate the raster points
    for z_rel in heights_rel:
        attempt = 0
        while True:
            attempt += 1
            height_started = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            # csv for different heights
            z_mm = int(round(z_rel * 1000.0))
            height_tag = f"z{z_mm}mm"
            #retry_tag = "" if attempt == 1 else f"_retry{attempt-1}"
            csv_tag = f"_{attempt}"
            csv_path = os.path.join(results_dir, f"{basename}_{height_tag}{csv_tag}.csv")
            with open(csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "x","y","z","reachable","fraction","radial_distance",
                    "time_start","time_end","ocr_series_csv","n_captures"
                ])
            points_total = 0
            points_reached = 0

            # ===================================
            # 1) RegulÃ¤res Raster wie konfiguriert (CSV + Marker)
            # ===================================
            # grid to drive serpentines
            for i in range(raster_size):                             # x: 0 ... N-1
                j_iter = range(raster_size) if (i % 2 == 0) else range(raster_size -1, -1, -1)
                for j in j_iter:                                     # y: oben->unten bzw. unten->oben
                    points_total += 1
                    z_abs = z_surface_top + z_rel
                    probe_tip = np.array([
                        x_start + i * delta,                         # x
                        y_start + j * delta,                         # y (j=N-1 ist "oben")
                        z_abs
                    ])

                    quat = base_quat  # no angles anymore
                    pose_goal = compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length)

                    # plan trajector ( cartesian path )
                    commander.set_path_constraints(ctx.build_constraints(quat))
                    commander.set_start_state_to_current_state()
                    plan, fraction = commander.compute_cartesian_path(
                        [pose_goal],
                        eef_step=0.005,
                        avoid_collisions=True,
                        jump_threshold=0.0   # optional
                    )
                    commander.clear_path_constraints()

                    # distance to center of the gamma source/ OberflÃ¤che der gammawuelle
                    radial_dist = np.linalg.norm([
                        probe_tip[0] - bx,
                        probe_tip[1] - by,
                        probe_tip[2] - z_surface_top
                    ])

                    if fraction > 0.8:
                        start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        commander.execute(plan, wait=True)

                        dir_name = f"{height_tag}_{attempt}"
                        point_dir = os.path.join(screenshots_root, dir_name)
                        os.makedirs(point_dir, exist_ok=True)
                        series_prefix = f"{basename}_{height_tag}_{dir_name}"
                        try:
                            ocr_csv_path, n_caps = capture_ocr_series(
                                duration_s=measurement_time_seconds, interval_s=0.5,
                                region=screen_region, out_dir=point_dir, basename_prefix=series_prefix
                            )
                        except Exception as e:
                            rospy.logwarn(f"[OCR] Fehler bei Serie: {e}")
                            ocr_csv_path, n_caps = "", 0

                        end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        points_reached += 1

                        with open(csv_path, "a", newline="") as f:
                            w = csv.writer(f)
                            w.writerow([
                                probe_tip[0], probe_tip[1], z_rel,
                                True, round(fraction, 2), round(radial_dist, 5),
                                start_time, end_time, ocr_csv_path, n_caps
                            ])

                        probe_marker.points.append(Point(*probe_tip))
                        marker_pub.publish(probe_marker)
                        rospy.loginfo(f"[âœ“] {height_tag} pt: x={probe_tip[0]:.3f}, y={probe_tip[1]:.3f}, z={z_rel:.3f} (z_abs={probe_tip[2]:.3f},fraction={fraction:.2f})")

                    else:
                        with open(csv_path, "a", newline="") as f:
                            w = csv.writer(f)
                            w.writerow([
                                probe_tip[0], probe_tip[1], z_rel,
                                False, round(fraction, 2), round(radial_dist, 5),
                                "", "", "", 0
                            ])
                        rospy.logwarn(f"[âœ—] {height_tag} NICHT erreichbar (fraction={fraction:.2f})")

            auto_success = (points_reached == points_total)

            print("\n--- heicht z ---")
            print(f"Height z={z_rel:.3f} m")
            print(f"Reached: {points_reached}/{points_total}")
            if auto_success:
                print("Result: Reached all points successfully")
            else:
                print("Result: Not all points reached successfully")
 

            rospy.loginfo(f"[INFO] CSV for z={z_rel:.3f} m saved: {csv_path}")

            # Optionen: [j] wiederholen, [n] weiter, [a] abbrechen
            while True:
                ans = input("Action? [y] Repeat / [n] Continue with next height / [c] Cancel: ").strip().lower()
                if ans in ('j', 'y', 'ja','yes'):
                    # repeat sam eheight again (neue CSV mit _retryN)
                    break_loop = False
                    break  # back to while-True-loop -> attempt+1
                elif ans in ('n', 'w', 'weiter','next'):
                    # next height 
                    break_loop = True
                    break
                elif ans in ('a', 'abbrechen', 'q','c'):
                    rospy.loginfo("Canceling raster scan, move to safe pose.")
                    move_to_safe_pose(commander)
                    return  
                else:
                    print("Invalid input, please enter [j] to repeat, [n] to continue with next height, or [a] to cancel.")

            if break_loop:
                # exit true while loop and continue with next height
                break

    rospy.loginfo("Move to safe position.")
    move_to_safe_pose(commander)
 
def raster_scan_safety(ctx, config, marker_pub, source_pose, source_size, probe_length=0.1):
    """
    raster scan for low heights (e.g. 2mm) with safety measures
    Rasterfahrt mit Upâ€“Overâ€“Down-Transfers:
      - Down:   senkrecht auf MesshÃ¶he (z) am Punkt
      - Up:     senkrecht in sichere Hover-HÃ¶he Ã¼ber dem Punkt
      - Over:   lateral auf Hover-HÃ¶he zum nÃ¤chsten Punkt
      - Down:   wieder auf MesshÃ¶he

    Erfordert:
      - compute_effector_pose_for_probe_tip(tip_xyz, quat, probe_length)
      - ensure_results_dir()
      - move_to_safe_pose(commander)
      - ctx: MoveGroupCommander + PlanningSceneInterface
    """
    scene = ctx.scene
    commander = ctx.commander

    # --------------------
    # Logging-Dateien
    # --------------------
    results_dir  = ensure_results_dir()
    screenshots_root = os.path.join(results_dir, "screenshots")
    run_ts       = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    basename     = f"raster_log_{run_ts}"
    summary_path = os.path.join(results_dir, f"{basename}_summary.csv")
    if not os.path.exists(summary_path):
        with open(summary_path, "w", newline="") as fsum:
            csv.writer(fsum).writerow([
                "z", "attempt", "points_total", "points_reached", "reach_fraction",
                "delta", "raster_size", "auto_success", "started_at", "finished_at", "csv_path"
            ])

    # --------------------
    # RViz Marker
    # --------------------
    probe_marker = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD)
    probe_marker.scale.x = probe_marker.scale.y = probe_marker.scale.z = 0.02
    probe_marker.color.b = 1.0
    probe_marker.color.a = 1.0
    probe_marker.header.frame_id = "panda_link0"
    probe_marker.points = []


    # --------------------
    # Konfiguration / Defaults
    # --------------------
    base_quat = quaternion_from_euler(-np.pi / 2, 0, 0)

    raster_size       = int(config.get("raster_size"))
    delta             = float(config.get("delta"))
    heights_rel           = list(config.get("heights"))
    align_time_seconds     = float(config.get("align_time_seconds", 2.0))        # Verweilen am Messpunkt
    lift              = float(config.get("lift", 0.015))    # min. zusÃ¤tzl. Hub Ã¼ber MesshÃ¶he
    hover_over_surface= float(config.get("hover_over_surface", 0.01)) # min. Abstand Ã¼ber Oberseite Quelle
    route_via_center  = bool(config.get("route_via_center", False))   # True => immer via (bx,by) schweben

    #ocr
    # OCR/Messdauer
    measurement_time_seconds = int(config.get("measurement_time_seconds",t_1 )) #60
    ocr_interval  = float(config.get("ocr_interval_s", 0.25)) # 1Hz ein bild pro sekunde; jeztzt  mit 0.25 =2 Hz 
    enable_ocr    = bool(config.get("enable_ocr", True))
    ocr_region    = config.get("ocr_region", screen_region)

    width = (raster_size - 1) * delta

    bx, by, bz = source_pose.pose.position.x, source_pose.pose.position.y, source_pose.pose.position.z
    x_start = bx 
    y_start = by 

    # geometrie der gammawuelle / kritischer bereich 
    radius_cyl    = source_size[0] / 2.0               # source_size = (Durchmesser_x, Durchmesser_y, HÃ¶he)
    z_surface_top =  source_size[2]  
    rospy.sleep(2)


    # --------------------
    # Helfer: Posen & Trajektorie planen
    # --------------------
    #def make_pose_for_tip(tip_xyz, quat):
        #return compute_effector_pose_for_probe_tip(np.array(tip_xyz), quat, probe_length)
    def hover_z_for(z_meas: float) -> float:
        return max(z_meas + lift, z_surface_top + hover_over_surface)

    def plan_exec_waypoints(quat, waypoints_xyz, label="", probe_length=0.1):
        """
        Versuche erst als Gesamtkette zu planen; falls unvollstÃ¤ndig,
        plane Segment-weise robuster nach.
        fhrt hoch, seitlich und dan runter
        """
        #poses = [make_pose_for_tip(wp, quat) for wp in waypoints_xyz]
        poses = [
        compute_effector_pose_for_probe_tip(np.array(wp), quat, probe_length)
        for wp in waypoints_xyz
        ]
        
        commander.set_start_state_to_current_state()
        plan, frac = commander.compute_cartesian_path(
            poses, eef_step=0.005, avoid_collisions=True, jump_threshold=0.0
        )
        if frac > 0.8:
            commander.execute(plan, wait=True)
            return True, frac

    # --------------------
    # Hauptschleife: pro HÃ¶he z
    # --------------------
    for z_rel in heights_rel:
        attempt = 0
        while True:
            attempt += 1
            height_started = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            # CSV pro HÃ¶he
            z_mm = int(round(z_rel * 1000.0))
            height_tag = f"z{z_mm}mm"
            csv_tag  =f"_{attempt}"
            csv_path   = os.path.join(results_dir, f"{basename}_{height_tag}{csv_tag}.csv")

            with open(csv_path, "w", newline="") as f:
                csv.writer(f).writerow(["x","y","z","reachable","fraction","radial_distance",
                                        "t_start","t_end","ocr_series_csv","n_captures"])

            points_total   = 0
            points_reached = 0

            # Rasterpunkte in gewÃ¼nschter Reihenfolge aufbauen (optional serpentin)
            grid_pts = []
            for i in range(raster_size):
                j_iter = range(raster_size) if (i % 2 == 0) else range(raster_size -1, -1, -1)
                probe_tip = []

                for j in j_iter:
                    points_total += 1
                    z_abs = z_surface_top + z_rel
                    probe_tip.append([x_start + i * delta, y_start + j * delta, z_abs])
                grid_pts.extend(probe_tip)

            # First raster : Overhead -> Down wenn es in kritischer hÃ¶he ist, sonst nicht 
            first_px, first_py, first_pz = grid_pts[0]
            first_hz = hover_z_for(first_pz)
            ok, frac = plan_exec_waypoints(base_quat,
                [[first_px, first_py, first_hz], [first_px, first_py, first_pz]],
                "approach_first"
            )
            if not ok:
                rospy.logwarn(f"[!] Erster Punkt nicht planbar (frac={frac:.2f}). Ãœberspringe diese HÃ¶he.")
                break # nÃ¤chste hÃ¶Ã¶he versiuchen 

            # --- Punkte sequenziell abarbeiten ---
            for idx, (px, py, pz) in enumerate(grid_pts):
                # Messung/Verweilen am aktuellen Punkt (wir sind hier bereits "Down")
                radial_dist = np.linalg.norm([px - bx, py - by, pz - z_surface_top])
                start_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                #point_dir     = os.path.join(screenshots_root, f"z{z:.3f}".replace('.','_'),
                                            #f"pt_{idx:03d}")
                #series_prefix = f"{basename}_z{z:.3f}_pt{idx:03d}".replace('.','_')
                probe_tip= (px,py,pz)
                dir_name = xy_dirname(probe_tip, unit="mm")
                point_dir = os.path.join(screenshots_root, height_tag, dir_name)
                series_prefix = f"{basename}_{height_tag}_{dir_name}"

                ocr_csv_path, n_caps = "", 0
                if enable_ocr:
                    try:
                        ocr_csv_path, n_caps = capture_ocr_series(
                            duration_s=measurement_time_seconds,
                            interval_s=ocr_interval,
                            region=screen_region,
                            out_dir=point_dir,
                            basename_prefix=series_prefix
                        )
                    except Exception as e:
                        rospy.logwarn(f"[OCR] Fehler bei Serie: {e}")
                else:
                    rospy.sleep(measurement_time_seconds)

                end_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                # Log/Marker
                points_total += 1
                points_reached += 1
                with open(csv_path, "a", newline="") as f:
                    csv.writer(f).writerow([px, py, z_rel, True, 1.0, round(radial_dist,4),
                                            start_time_str, end_time_str, ocr_csv_path, n_caps])

                probe_marker.points.append(Point(px, py, pz))
                marker_pub.publish(probe_marker)
                rospy.loginfo(f"[âœ“] {height_tag} pt {idx+1}/{len(grid_pts)}: x={px:.3f}, y={py:.3f}, z={z_rel:.3f},(z_abs={pz:.3f}")


                # Transfer zum nÃ¤chsten Punkt (Upâ€“Overâ€“Down)
                if idx < len(grid_pts) - 1:
                    nx, ny, nz = grid_pts[idx + 1]
                                    
                hz_cur  = hover_z_for(pz)
                hz_next = hover_z_for(nz)
                if route_via_center:
                    waypoints = [[px, py, hz_cur], [bx, by, hz_cur], [nx, ny, hz_next], [nx, ny, nz]]
                else:
                    waypoints = [[px, py, hz_cur], [nx, ny, hz_next], [nx, ny, nz]]
                ok_move, frac_move = plan_exec_waypoints(base_quat, waypoints, "transfer")
                if not ok_move:
                        rospy.logwarn(f"[âœ—] Transfer nicht planbar (frac={frac_move:.2f}). Ãœberspringe Zielpunkt.")
                        continue


            # --- Zusammenfassung pro HÃ¶he ---
            height_finished = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            reach_fraction  = (points_reached / float(points_total)) if points_total else 0.0
            auto_success    = (points_reached == points_total)

            print("\n--- hight-Zwischenbilanz ---")
            print(f"hight z={z_rel:.3f} m")
            print(f"Erreicht: {points_reached}/{points_total}  (fraction={reach_fraction:.2%})")
            print("Ergebnis:", "ERFOLGREICH" if auto_success else "NICHT vollstÃ¤ndig")

            with open(summary_path, "a", newline="") as fsum:
                csv.writer(fsum).writerow([
                    z_rel, attempt, points_total, points_reached, round(reach_fraction, 4),
                    delta, raster_size, auto_success, height_started, height_finished, csv_path
                ])
            rospy.loginfo(f"[INFO] CSV für z={z_rel:.3f} m gespeichert: {csv_path}")

            # Nutzerentscheidung wie gehabt
            while True:
                ans = input("Aktion? [j] wiederholen / [n] weiter / [a] abbrechen: ").strip().lower()
                if ans in ('j', 'y', 'ja'):
                    break_loop = False
                    break
                elif ans in ('n', 'w', 'weiter'):
                    break_loop = True
                    break
                elif ans in ('a', 'abbrechen', 'q'):
                    rospy.loginfo("Vom Nutzer abgebrochen. Fahre in sichere Pose¦")
                    move_to_safe_pose(commander)
                    return
                else:
                    print("UngÃ¼ltige Eingabe. Bitte j/n/a.")
 
            if break_loop:
                break  # nÃ¤chste HÃ¶he

    rospy.loginfo("Fahre in sichere Pose zurück â€¦")
    move_to_safe_pose(commander)


def raster_menu(ctx):
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    source_pose, source_size = setup_environment(ctx.scene, ctx.commander)
 
    while not rospy.is_shutdown():
        print("\n=== Raster-Optionen ===")
        print("0: Alignment of the soure(manual).")
        print("1: Testraster für genauigkeig (raster_size=2, heights=[0.3], delta=0.05)")
        print("2: TEST! Messraster z>0.05 (raster_size=2, heights=[0.05, 0.1, 0.2], delta=0.05)")
        print("3: Messraster z< 0.05(raster_size=35, heights=[0.01,0.02,0.03,0.04], delta=0.002)")
        print("q: Beenden")
        raster_choice = input("wähle (0/1/2/3/q): ").strip().lower()
        if raster_choice == 'q':
            print("Beende Raster-Demo.")
            break

        elif raster_choice not in ['0','1','2','3','4','5']:
            print("Ungültige Eingabe. Bitte erneut versuchen.")
            continue

        repeat = True
        while repeat and not rospy.is_shutdown():

            if raster_choice == '0':
                ok = source_alignment(
                    ctx,
                    source_pose,
                    source_size,
                    probe_length=0.1,       # ggf. anpassen
                    hover_over_surface=0.015, # z.B. 10 mm über Oberkante######anpassen todo
                    #dwell_s=t_0              # z.B. 10 s Wartezeit
                )
                source_alignment(ctx, source_pose, source_size, probe_length=0.1,
                     hover_over_surface=0.02)
            """
            if raster_choice == '1':
                config = {
                    "raster_size": 2,
                    "delta": 0.005,
                    "heights": [0.05, 0.003],
                }
                raster_scan_1(ctx, config, marker_pub, source_pose, source_size)
                #raster_scan1 ist raster
            """


            if raster_choice == '1':
                config = {
                    "raster_size": 31,             #  31Anzahl Punkte
                    "delta": 0.002,                 # in m
                    "heights": [0.001, 0.005, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09],  #[0.025, 0.03, 0.04, 0.05, 0.06, 0.07],     # 0.001, 0.002, 0.003, 0.004 in mm
                    "measurement_time_seconds": 13, # Messdauer pro Punkt
                    "ocr_interval_s": 0.5,        # 4 Hz OCR
                    "enable_ocr": True,
                }
                raster_scan(ctx, config, marker_pub, source_pose, source_size)
                # Nullmessung: 1,0.05
                #das ist linienraster'
                # 0.054- 0.035 = 0.019

            if raster_choice == '2':
                config = {
                    "raster_size": 1,             # Anzahl Punkte
                    "delta": 0.002,                 # in m
                    "heights": [0.05],     # 0.04, 0.05, 0.06, 0.07
                    "measurement_time_seconds": 20, # Messdauer pro Punkt
                    "ocr_interval_s": 0.5,        # 4 Hz OCR
                    "enable_ocr": True,
                }
                raster_scan(ctx, config, marker_pub, source_pose, source_size)
                # nULLMESSUNG: 1,0.05
                #das ist linienraster'

            if raster_choice == '3':
                config = {
                    "raster_size": 26,             # Anzahl Punkte
                    "delta": 0.002,                 # in m
                    "heights": [0.009, 0.01, 0.02, 0.03],     # 0.04, 0.05, 0.06, 0.07
                    "measurement_time_seconds": 13, # Messdauer pro Punkt
                    "ocr_interval_s": 0.5,        # 4 Hz OCR
                    "enable_ocr": True,
                }
                raster_scan(ctx, config, marker_pub, source_pose, source_size)    

            if raster_choice == '4':
                config = {
                    "raster_size": 31,             # Anzahl Punkte
                    "delta": 0.002,                 # in m
                    "heights": [0.04, 0.05, 0.06, 0.07],   # 5 mm und 10 mm über Quelle
                    "measurement_time_seconds": 13, # Messdauer pro Punkt
                    "ocr_interval_s": 0.5,        # 4 Hz OCR
                    "enable_ocr": True,
                }
                raster_scan(ctx, config, marker_pub, source_pose, source_size)
                # nULLMESSUNG: 1,0.05
                #das ist linienraster'


            if raster_choice == '5':
                config = {
                    "raster_size": 26,             # Anzahl Punkte
                    "delta": 0.004,                 # in m
                    "heights": [0.08, 0.09, 0.1, 0.12],     # 5 mm und 10 mm über Quelle
                    "measurement_time_seconds": 13, # Messdauer pro Punkt
                    "ocr_interval_s": 0.5,        # 4 Hz OCR
                    "enable_ocr": True,
                }
                raster_scan(ctx, config, marker_pub, source_pose, source_size)
                # nULLMESSUNG: 1,0.05
                #das ist linienraster'


            if raster_choice == '6':
                config = {
                    "raster_size": 6,
                    "delta": 0.003,
                    "heights": [0.001],       # 2 mm
                    "measurement_time_seconds": t_1,
                    "align_time_seconds": 2.0,
                    "lift_clearance": 0.01,  # 15 mm Hub Ã¼ber MesshÃ¶he
                    "hover_over_surface": 0.05,
                    "route_via_center": False,
                    "serpentine": True
                }
                raster_scan_safety(ctx, config, marker_pub, source_pose, source_size)

            again = input("\n same raster again(yes/no)? ").strip().lower()
            if again != 'yes':
                repeat = False
 

 

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('raster_scan_demo')
    ctx = Context('panda_arm')
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    raster_menu(ctx)
