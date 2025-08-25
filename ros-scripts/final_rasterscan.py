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

 
node_prefix = 'raster_scan_demo: '
timeout = 10.0
t_0 = 4 # time to align the gamma source
t_1 = 4  # time for countingt the cps of the gamma source

#ocr
pytesseract.pytesseract.tesseract_cmd = os.environ.get("TESSERACT_CMD", "/usr/bin/tesseract")

# #OBS muss oben in die linke ecke plaziert werden, mit der usprungsgröße wenn ich es lffne, dann ist die REGIOn richtig ausgerichtet 
REGION = {"top": 100, "left": 300, "width": 800, "height": 500}

 

#eingie links: https://projects.saifsidhik.page/franka_ros_interface/DOC.html#module-franka_moveit

 
#TODO
    # keine winkel mehr 
    # der panda soll immer nur ebenen Fahren , also Z=x und danach einmal das script abspeichern mit dem namen der höhe 
    # im script soll dann der ortsverkotr der sonde mit angegegben werden, da in dieser Art deer winkel eingegebe wird 
    # nur halbquadrate fahren um die boxpose, also boxpose lieg am rand 
    # hohenangabe anpassen ; anfahrende höhe im paonda so lassen, aber offsetz dann in der csv apassen

 

    
class Context:
    def __init__(self, move_group_name):
        rospy.loginfo(node_prefix + 'Waiting for move_group/status')
        rospy.wait_for_message('move_group/status', actionlib_msgs.msg.GoalStatusArray, timeout)

 

        self.commander = moveit_commander.MoveGroupCommander(move_group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

 

        self.commander.set_end_effector_link('panda_sensor_mount') # panda_link8
        self.commander.set_planning_time(30) 
        self.commander.set_num_planning_attempts(3) 
        
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
    """
    This function moves the robot to a predefined and safe position
    """
    # get current joint values
    joint_goal = commander.get_current_joint_values()

 

    # set safe joint values (tested in Rviz)
    joint_goal[0] = 0.0                  # panda_joint1
    joint_goal[1] = -np.pi / 4           # panda_joint2
    joint_goal[2] = 0.0
    joint_goal[3] = -np.pi*3 / 4         # 135°
    joint_goal[4] = 0.0
    joint_goal[5] = np.pi / 2            # 90°
    joint_goal[6] = np.pi / 4            # 45°

 

    rospy.loginfo("Move to safe position ...")
    commander.go(joint_goal, wait=True)
    commander.stop()
    rospy.loginfo("Reached safe position.")

def compute_effector_pose_for_probe_tip(probe_tip, quat, probe_length=0.1):
    """
    This function computes the pose of the end probe tip based on the endeffector pose.
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
    This function sets up the environment for the raster scan.
    It adds a table, a wall, four legs, and a box (cylinder) to the scene to avoide collisions.
    """
    rospy.sleep(2)
    """
    #rastergenauigkeit testen
    rasterpoint0 = PoseStamped()
    rasterpoint0.header.frame_id = commander.get_planning_frame()
    rasterpoint0.pose.orientation.w = 1.0
    rasterpoint0.pose.position.x = 0.35
    rasterpoint0.pose.position.y = 0.0
    rasterpoint0.pose.position.z = 0.0001
    scene.add_cylinder("rasterpoint0", rasterpoint0, 0.005,0.005)

 

    rasterpoint1 = PoseStamped()
    rasterpoint1.header.frame_id = commander.get_planning_frame()
    rasterpoint1.pose.orientation.w = 1.0
    rasterpoint1.pose.position.x = 0.35
    rasterpoint1.pose.position.y = 0.05
    rasterpoint1.pose.position.z = 0.0001
    scene.add_cylinder("rasterpoint1", rasterpoint1, 0.005,0.005)

 

    rasterpoint2 = PoseStamped()
    rasterpoint2.header.frame_id = commander.get_planning_frame()
    rasterpoint2.pose.orientation.w = 1.0
    rasterpoint2.pose.position.x = 0.4
    rasterpoint2.pose.position.y = 0.05
    rasterpoint2.pose.position.z = 0.0001
    scene.add_cylinder("rasterpoint2", rasterpoint2, 0.005,0.005)
    """

 

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
    radius = 0.01 # 1cm radius, durchmesser 2cm
    height_strahler = 0.01
    scene.add_cylinder("small_cyl", box_pose, height_strahler, radius)
    smal_circ_size=(2*radius, 2*radius,height_strahler)

 

    rospy.sleep(2)
    return box_pose, smal_circ_size


def ensure_results_dir():
    res_dir = os.path.join(os.path.expanduser("~"), "catkin_ws", "raster_results")
    os.makedirs(res_dir, exist_ok=True)
    return res_dir

def source_alignment(ctx, box_pose, box_size, probe_length=0.1, height_strahler=0.01,
                     hover_over_surface=0.02):

    commander = ctx.commander
    base_quat = quaternion_from_euler(-np.pi/2, 0, 0)  # +Y (Sonde) zeigt nach -Z (nach unten)

    z_alignment = height_strahler + 0.01 # hight for source alignemnt
    bx, by, bz = box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z
    tip_alignment = np.array([bx, by, z_alignment])  # center of the source

    pose_over_cyl = compute_effector_pose_for_probe_tip(tip_alignment, base_quat, probe_length)

    commander.set_start_state_to_current_state()
    plan_over, frac_over = commander.compute_cartesian_path(
        [pose_over_cyl], eef_step=0.005, avoid_collisions=True, jump_threshold=0.0
    )
    commander.execute(plan_over, wait=True)
    rospy.sleep(t_0)  
    rospy.loginfo(f"[✓] Approach: über Kreiszentrum bei z={z_alignment:.3f} m")
    rospy.loginfo(f"Stop the robot to align the gamma source for.")

    ### reset einbauen !!


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
            ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            img_path = os.path.join(out_dir, f"{basename_prefix}_{ts}_{idx:03d}.png")
            img_scaled.save(img_path)

            idx += 1
            w.writerow([idx, datetime.now().isoformat(), round(now - start, 3), ocr_time, ocr_date, text, img_path])

            # präzises Intervall
            next_tick += interval_s
            sleep_for = max(0.0, next_tick - time.monotonic())
            time.sleep(sleep_for)

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



def raster_scan(ctx, config, marker_pub, box_pose, box_size, probe_length=0.1):
    """
    Grid scan 
    
    """
    scene = ctx.scene
    commander = ctx.commander

    results_dir = ensure_results_dir()

    screenshots_root = os.path.join(results_dir, "screenshots")  

    run_ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]## bessere auflösung ( in ms) 
    basename = f"raster_log_{run_ts}"
    summary_path = os.path.join(results_dir, f"{basename}_summary.csv")

    # Summary-CSV vorbereiten (einmal pro Run)
    if not os.path.exists(summary_path):
        with open(summary_path, "w", newline="") as fsum:
            w = csv.writer(fsum)
            w.writerow(["z", "attempt", "points_total", "points_reached", "reach_fraction",
                         "delta", "raster_size", "auto_success", "started_at", "finished_at", "csv_path"])

 

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
    heights = list(config["heights"])

    width = (raster_size - 1) * delta

    # Messdauer/Intervalle für OCR
    measurement_time_seconds = int(config.get("measurement_time_seconds", 9)) #60
    # in raster_scan(), direkt nach dem Einlesen der config:
    ocr_interval  = float(config.get("ocr_interval_s", 1.0))
    enable_ocr    = bool(config.get("enable_ocr", True))
    ocr_region    = config.get("ocr_region", REGION)  # erlaubt Override über config


    # center of gamma source
    bx, by, bz = box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z
    # startpoint: upper left corner
    x_start = bx 
    y_start = by 

    rospy.sleep(2)

    # main loof do generate the raster points
    for z in heights:
        attempt = 0
        while True:
            attempt += 1
            height_started = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            # csv for different heights
            z_mm = int(round(z * 1000.0))
            height_tag = f"z{z_mm}mm"
            #height_tag = f"z{z:.3f}mm".replace('.', '_')
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

            # ===================================
            # 1) Reguläres Raster wie konfiguriert (CSV + Marker)
            # ===================================
            """
            for i in range(raster_size):
                for j in range(raster_size):
                    points_total += 1

 

                    probe_tip = np.array([
                        x_start + i * delta,
                        y_start + j * delta,
                        z
                    ])
            """
            # grid to drive serpentines
            for i in range(raster_size):                             # x: 0 ... N-1
                j_iter = range(raster_size) if (i % 2 == 0) else range(raster_size -1, -1, -1)
                for j in j_iter:                                     # y: oben->unten bzw. unten->oben
                    points_total += 1

                    probe_tip = np.array([
                        x_start + i * delta,                         # x
                        y_start + j * delta,                         # y (j=N-1 ist "oben")
                        z
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

                    # distance to center of the gamma source
                    radial_dist = np.linalg.norm([
                        probe_tip[0] - bx,
                        probe_tip[1] - by,
                        probe_tip[2] - bz
                    ])

                    if fraction > 0.8:
                        start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        commander.execute(plan, wait=True)
                        #rospy.sleep(t_1)
                        #point_dir = os.path.join(screenshots_root, height_tag, f"pt_i{i:02d}_j{j:02d}")
                        #series_prefix = f"{basename}_{height_tag}_i{i:02d}_j{j:02d}"
                        dir_name = xy_dirname(probe_tip, unit="mm")
                        point_dir = os.path.join(screenshots_root, height_tag, dir_name)
                        series_prefix = f"{basename}_{height_tag}_{dir_name}"
                        try:
                            ocr_csv_path, n_caps = capture_ocr_series(
                                duration_s=measurement_time_seconds, interval_s=1.0,
                                region=REGION, out_dir=point_dir, basename_prefix=series_prefix
                            )
                        except Exception as e:
                            rospy.logwarn(f"[OCR] Fehler bei Serie: {e}")
                            ocr_csv_path, n_caps = "", 0

                        end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
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


            # finished for heicht z: save in csv files (a summary and seperate) ---
            height_finished = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            reach_fraction = (points_reached / float(points_total)) if points_total else 0.0
            auto_success = (points_reached == points_total)

            print("\n--- heicht z ---")
            print(f"Height z={z:.3f} m")
            print(f"Reached: {points_reached}/{points_total}  (fraction={reach_fraction:.2%})")
            if auto_success:
                print("Result: Reached all points successfully")
            else:
                print("Result: Not all points reached successfully")

            with open(summary_path, "a", newline="") as fsum:
                w = csv.writer(fsum)
                w.writerow([z, attempt, points_total, points_reached, round(reach_fraction, 4),
                             delta, raster_size, auto_success, height_started, height_finished, csv_path])

            rospy.loginfo(f"[INFO] CSV for z={z:.3f} m saved: {csv_path}")

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

def raster_scan_safety(ctx, config, marker_pub, box_pose, box_size, probe_length=0.1):
    """
    raster scan for low heights (e.g. 2mm) with safety measures
    Rasterfahrt mit Up–Over–Down-Transfers:
      - Down:   senkrecht auf Messhöhe (z) am Punkt
      - Up:     senkrecht in sichere Hover-Höhe über dem Punkt
      - Over:   lateral auf Hover-Höhe zum nächsten Punkt
      - Down:   wieder auf Messhöhe

 

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
    heights           = list(config.get("heights"))
    align_time_seconds     = float(config.get("align_time_seconds", 2.0))        # Verweilen am Messpunkt
    lift              = float(config.get("lift", 0.015))    # min. zusätzl. Hub über Messhöhe
    hover_over_surface= float(config.get("hover_over_surface", 0.01)) # min. Abstand über Oberseite Quelle
    route_via_center  = bool(config.get("route_via_center", False))   # True => immer via (bx,by) schweben
    
    # für die Quellengeometrie und schutzkreis
    protect_margin   = float(config.get("protect_margin", 0.005))  # +5 mm
    low_height_band  = float(config.get("low_height_band", 0.010)) # kritisch bis 10 mm über Oberfläche

    #ocr
    # OCR/Messdauer
    measurement_time_seconds = int(config.get("measurement_time_seconds", )) #60
    ocr_interval  = float(config.get("ocr_interval_s", 1.0))
    enable_ocr    = bool(config.get("enable_ocr", True))
    ocr_region    = config.get("ocr_region", REGION)

    width = (raster_size - 1) * delta

    bx, by, bz = box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z
    x_start = bx 
    y_start = by 

    # geometrie der gammawuelle / kritischer bereich 
    radius_cyl    = box_size[0] / 2.0               # box_size = (Durchmesser_x, Durchmesser_y, Höhe)
    z_surface_top = bz + box_size[2]  
    rospy.sleep(2)


 
    # --------------------
    # Helfer: Posen & Trajektorie planen
    # --------------------
    #def make_pose_for_tip(tip_xyz, quat):
        #return compute_effector_pose_for_probe_tip(np.array(tip_xyz), quat, probe_length)
    def hover_z_for(z_meas: float) -> float:
        return max(z_meas + lift, z_surface_top + hover_over_surface)
    
    def dist_point_to_segment(cx, cy, ax, ay, bx_, by_):
        """Minimaler Abstand des Punkts C zum Linienstück A->B in XY."""
        abx, aby = (bx_ - ax), (by_ - ay)
        acx, acy = (cx - ax), (cy - ay)
        ab2 = abx*abx + aby*aby
        if ab2 == 0.0:
            return np.hypot(acx, acy)
        t = max(0.0, min(1.0, (acx*abx + acy*aby) / ab2))
        px = ax + t*abx
        py = ay + t*aby
        return np.hypot(cx - px, cy - py)

 

    def needs_up_over_down(px, py, pz, nx, ny, nz):
        """True, wenn P->N in XY den Schutzkreis schneidet und die Höhe 'kritisch' niedrig ist."""
        if max(pz, nz) > z_surface_top + low_height_band:
            return False
        r_protect = radius_cyl + protect_margin
        d = dist_point_to_segment(bx, by, px, py, nx, ny)
        return d <= r_protect

 

    def plan_exec_waypoints(quat, waypoints_xyz, label="", probe_length=0.1):
        """
        Versuche erst als Gesamtkette zu planen; falls unvollständig,
        plane Segment-weise robuster nach.
        fährt hoch, seitlich und dan runter
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
        '''
        ok_all = True
        min_frac = 1.0
        for a, b in zip(poses[:-1], poses[1:]):
            commander.set_start_state_to_current_state()
            plan_seg, frac_seg = commander.compute_cartesian_path(
                [a, b], eef_step=0.005, avoid_collisions=True#, jump_threshold=0.0
            )
            min_frac = min(min_frac, frac_seg)
            if frac_seg > 0.8:
                commander.execute(plan_seg, wait=True)
            else:
                ok_all = False
                break
        return ok_all, min_frac
        '''

    # --------------------
    # Hauptschleife: pro Höhe z
    # --------------------
    for z in heights:
        attempt = 0
        while True:
            attempt += 1
            height_started = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

 

            # CSV pro Höhe
            z_mm = int(round(z * 1000.0))
            height_tag = f"z{z_mm}mm"
            #height_tag = f"z{z:.3f}m".replace('.', '_')
            retry_tag  = "" if attempt == 1 else f"_retry{attempt-1}"
            csv_path   = os.path.join(results_dir, f"{basename}_{height_tag}{retry_tag}.csv")

            with open(csv_path, "w", newline="") as f:
                csv.writer(f).writerow(["x","y","z","reachable","fraction","radial_distance",
                                        "t_start","t_end","ocr_series_csv","n_captures"])

            points_total   = 0
            points_reached = 0

            # Rasterpunkte in gewünschter Reihenfolge aufbauen (optional serpentin)
            grid_pts = []
            for i in range(raster_size):
                j_iter = range(raster_size) if (i % 2 == 0) else range(raster_size -1, -1, -1)
                probe_tip = []

                for j in j_iter:
                    points_total += 1
                    probe_tip.append([x_start + i * delta, y_start + j * delta, z])
                grid_pts.extend(probe_tip)

            # First raster : Overhead -> Down wenn es in kritischer höhe ist, sonst nicht 
            first_px, first_py, first_pz = grid_pts[0]
            if needs_up_over_down(first_px, first_py, first_pz, first_px, first_py, first_pz):
                first_hz = hover_z_for(first_pz)
                ok, frac = plan_exec_waypoints(base_quat, [[first_px, first_py, first_hz],
                                                           [first_px, first_py, first_pz]], "approach_first")
            else:
                ok, frac = plan_exec_waypoints(base_quat, [[first_px, first_py, first_pz]], "approach_first")

            if not ok:
                rospy.logwarn(f"[!] Erster Punkt nicht planbar (frac={frac:.2f}). Überspringe diese Höhe.")
                break # nächste hööhe versiuchen 

            # --- Punkte sequenziell abarbeiten ---
            for idx, (px, py, pz) in enumerate(grid_pts):
                # Messung/Verweilen am aktuellen Punkt (wir sind hier bereits "Down")
                radial_dist = np.linalg.norm([px - bx, py - by, pz - bz])
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
                            region=REGION,
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
                    csv.writer(f).writerow([px, py, pz, True, 1.0, round(radial_dist,4),
                                            start_time_str, end_time_str, ocr_csv_path, n_caps])

                probe_marker.points.append(Point(px, py, pz))
                marker_pub.publish(probe_marker)
                rospy.loginfo(f"[✓] {height_tag} pt {idx+1}/{len(grid_pts)}: x={px:.3f}, y={py:.3f}, z={pz:.3f}")

 

                # Transfer zum nächsten Punkt (Up–Over–Down)
                if idx < len(grid_pts) - 1:
                    nx, ny, nz = grid_pts[idx + 1]
                    
                    if needs_up_over_down(px, py, pz, nx, ny, nz):
                        # Nur über/nahe dem Zylinder: Up–Over–Down (optional via center)
                        hz_cur  = hover_z_for(pz)
                        hz_next = hover_z_for(nz)
                        if route_via_center:
                            waypoints = [[px, py, hz_cur], [bx, by, hz_cur], [nx, ny, hz_next], [nx, ny, nz]]
                        else:
                            waypoints = [[px, py, hz_cur], [nx, ny, hz_next], [nx, ny, nz]]
                    else:
                        # außerhalb Schutzkreis: direkte Fahrt auf Messhöhe
                        waypoints = [[nx, ny, nz]]

 

                    ok_move, frac_move = plan_exec_waypoints(base_quat, waypoints, "transfer")
                    if not ok_move:
                        rospy.logwarn(f"[✗] Transfer nicht planbar (frac={frac_move:.2f}). Überspringe Zielpunkt.")
                        continue

 

            # --- Zusammenfassung pro Höhe ---
            height_finished = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            reach_fraction  = (points_reached / float(points_total)) if points_total else 0.0
            auto_success    = (points_reached == points_total)

            print("\n--- Höhen-Zwischenbilanz ---")
            print(f"Höhe z={z:.3f} m")
            print(f"Erreicht: {points_reached}/{points_total}  (fraction={reach_fraction:.2%})")
            print("Ergebnis:", "ERFOLGREICH" if auto_success else "NICHT vollständig")

            with open(summary_path, "a", newline="") as fsum:
                csv.writer(fsum).writerow([
                    z, attempt, points_total, points_reached, round(reach_fraction, 4),
                    delta, raster_size, auto_success, height_started, height_finished, csv_path
                ])
            rospy.loginfo(f"[INFO] CSV für z={z:.3f} m gespeichert: {csv_path}")

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
                    rospy.loginfo("Vom Nutzer abgebrochen. Fahre in sichere Pose …")
                    move_to_safe_pose(commander)
                    return
                else:
                    print("Ungültige Eingabe. Bitte j/n/a.")

            if break_loop:
                break  # nächste Höhe

 

    rospy.loginfo("Fahre in sichere Pose zurück …")
    move_to_safe_pose(commander)


def raster_menu(ctx):
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    box_pose, box_size = setup_environment(ctx.scene, ctx.commander)
 
    while not rospy.is_shutdown():
        print("\n=== Raster-Optionen ===")
        print("0: Alignment of the soure(manual).")
        print("1: Testraster für genauigkeig (raster_size=2, heights=[0.3], delta=0.05)")
        print("2: Messraster z>0.05 (raster_size=35, heights=[0.05, 0.06, 0.07, 0.08,0.09,0.1,0.15,0.2,0.25,0.3], delta=0.002)")
        print("3: Messraster z< 0.05(raster_size=35, heights=[0.01,0.02,0.03,0.04], delta=0.002)")
        print("q: Beenden")
        raster_choice = input("Wähle (0/1/2/3/q): ").strip().lower()
        if raster_choice == 'q':
            print("Beende Raster-Demo.")
            break

 

        elif raster_choice not in ['0','1', '2','3','alignment']:
            print("Ungültige Eingabe. Bitte erneut versuchen.")
            continue

        repeat = True
        while repeat and not rospy.is_shutdown():

            if raster_choice == '0':
                ok = source_alignment(
                    ctx,
                    box_pose,
                    box_size,
                    probe_length=0.1,       # ggf. anpassen
                    hover_over_surface=0.02, # z.B. 20 mm über Oberkante
                    #dwell_s=t_0              # z.B. 10 s Wartezeit
                )

                source_alignment(ctx, box_pose, box_size, probe_length=0.1, height_strahler=0.01,
                     hover_over_surface=0.02)

            if raster_choice == '1':
                config = {
                    "raster_size": 1,
                    "delta": 0.05,
                    "heights": [0.07,0.03,0.02],
                }
                raster_scan(ctx, config, marker_pub, box_pose, box_size)

            if raster_choice == '2':
                config = {
                    "raster_size": 35,
                    "delta": 0.002,
                    "heights": [0.05, 0.06, 0.07, 0.08,0.09,0.1,0.15,0.2,0.25,0.3],
                }

                raster_scan(ctx, config, marker_pub, box_pose, box_size)


            if raster_choice == '3':
                config = {
                    "raster_size": 2,
                    "delta": 0.05,
                    "heights": [0.15,0.2,0.3],       # 2 mm
                    "measurement_time_seconds": 9.0,
                    "align_time_seconds": 2.0,
                    "lift_clearance": 0.02,  # 15 mm Hub über Messhöhe
                    "hover_over_surface": 0.05,
                    "route_via_center": False,
                    "serpentine": True
                }

                raster_scan_safety(ctx, config, marker_pub, box_pose, box_size)

            again = input("\nNochmal ausführen (ja/nein)? ").strip().lower()
            if again != 'ja':
                repeat = False
 

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('raster_scan_demo')
    ctx = Context('panda_arm')
    marker_pub = rospy.Publisher('raster_points_marker', Marker, queue_size=10)
    raster_menu(ctx)




