#!/usr/bin/env python3

import rospy
import subprocess
import os
import rospkg
import signal


def start_roscore():
    """
    Starts roscore as a subprocess and waits for it to initialize.
    Returns the subprocess object.
    """
    roscore_proc = subprocess.Popen(["roscore"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    # Wait a few seconds for roscore to come up; adjust as needed
    # time.sleep(3)
    return roscore_proc


def stop_roscore(roscore_proc):
    """
    Gracefully terminates the roscore subprocess.
    """
    # Send SIGINT to allow roscore to shut down cleanly
    roscore_proc.send_signal(signal.SIGINT)
    roscore_proc.wait()


def generate_urdf():
    """
    Generates a URDF from a xacro file and saves it to disk.
    The file paths are obtained from ROS parameters or set to defaults.
    """
    # Determine default paths using rospkg for the 'box_model' package
    default_pkg = "box_model"
    rospack = rospkg.RosPack()
    default_xacro = os.path.join(rospack.get_path(default_pkg), "urdf", "box", "box.urdf.xacro")
    default_urdf = os.path.join(rospack.get_path(default_pkg), "urdf", "box", "boxi.urdf")

    # Get parameters (if set on the parameter server) or use defaults
    xacro_file = rospy.get_param("~xacro_file", default_xacro)
    urdf_file = rospy.get_param("~urdf_file", default_urdf)

    rospy.loginfo("Generating URDF with the following settings:")
    rospy.loginfo("  xacro file: %s", xacro_file)
    rospy.loginfo("  output file: %s", urdf_file)

    # Execute the xacro command to generate the URDF
    try:
        cmd = ["xacro", xacro_file]
        output = subprocess.check_output(cmd, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as e:
        rospy.logerr("Error generating URDF from xacro:\n%s", e.output.decode())
        return

    # Save the generated URDF to disk
    try:
        with open(urdf_file, "wb") as f:
            f.write(output)
        rospy.loginfo("URDF successfully generated and saved to: %s", urdf_file)
    except IOError as e:
        rospy.logerr("Failed to write URDF to file: %s", e)


def main():
    # Start roscore as a subprocess
    roscore_proc = start_roscore()
    try:
        # Initialize the ROS node (this now connects to the roscore we just started)
        rospy.init_node("urdf_generator", anonymous=True)
        generate_urdf()
    finally:
        # Clean up: kill the roscore process
        stop_roscore(roscore_proc)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
