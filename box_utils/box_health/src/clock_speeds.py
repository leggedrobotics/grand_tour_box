import rospy
from box_health.msg import ClockFrequencies


def read_clock_info(file_path):
    clock_info = {}
    with open(file_path, "r") as file:
        for line in file:
            source, frequency = line.split(":")[0], line.split("=")[-1]
            clock_info[source] = int(frequency)

    return clock_info


def publish_clock_frequencies():
    namespace = rospy.get_namespace()
    rospy.init_node("clocks_publisher_pi")
    pub = rospy.Publisher(namespace + "health_status/pi_clock_frequencies", ClockFrequencies, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        frequencies = read_clock_info("/tmp/clock_info.txt")
        if "dpi" in frequencies:
            msg = ClockFrequencies()
            msg.header.stamp = rospy.Time.now()
            for clock, frequency in frequencies.items():
                setattr(msg, clock, frequency)

            pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_clock_frequencies()
    except rospy.ROSInterruptException:
        pass
