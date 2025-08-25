#!/usr/bin/env python3
import rospy, tf2_ros, numpy as np
from tf.transformations import quaternion_matrix

def main():
    rospy.init_node('tilt_measure')
    buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    lis = tf2_ros.TransformListener(buf)
    rate = rospy.Rate(2)

    world = "panda_link0"          # dein Welt-/Basisframe
    mount = "panda_sensor_mount"   # Frame am Tool (dort wo die Sonde „dranhängt“)

    rospy.sleep(0.5)
    while not rospy.is_shutdown():
        try:
            tr = buf.lookup_transform(world, mount, rospy.Time(0), rospy.Duration(1.0))
            q = tr.transform.rotation
            R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

            # lokale +Y-Achse des Mounts in Weltkoordinaten
            v = R.dot(np.array([0.0, 1.0, 0.0]))
            down = np.array([0.0, 0.0, -1.0])

            # Gesamt-Neigungswinkel zur idealen Down-Achse (−Z)
            eps = np.degrees(np.arccos(np.clip(np.dot(v, down) / (np.linalg.norm(v)*1.0), -1.0, 1.0)))

            # Kleine-Winkel-Komponenten (Vorzeichen beachten, siehe Kommentar):
            #  +eps_x  => Achse lehnt Richtung +Y (Kippung um Welt-X)
            #  +eps_y  => Achse lehnt Richtung −X (Kippung um Welt-Y)
            eps_x = np.degrees(np.arctan2(v[1], -v[2]))      # ~Kippung um X
            eps_y = np.degrees(-np.arctan2(v[0], -v[2]))     # ~Kippung um Y

            print(f"tilt ≈ {eps:.3f}°   (about X ≈ {eps_x:.3f}°, about Y ≈ {eps_y:.3f}°)   v_world={v}")
        except Exception as e:
            print("waiting for TF...", e)
        rate.sleep()

if __name__ == "__main__":
    main()
