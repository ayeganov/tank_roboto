#!/usr/bin/env python
import argparse
import functools
import os
os.sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages/")
os.sys.path.append("/home/aleks/space/.pyenv/versions/2.7.14/lib/python2.7/site-packages")
import signal
import time

import rospy
import zmq
from sensor_msgs.msg import Range
from std_msgs.msg import String


def callback(sensor_id, range_msg):
    print("I am sensor: {}".format(sensor_id))
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", range_msg)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('plotter_node', anonymous=True)

    sensor1_cb = functools.partial(callback, 1)
    rospy.Subscriber("sensor1", Range, sensor1_cb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def talker():
    pub = rospy.Publisher("sensor1", Range, queue_size=50)
    rospy.init_node("robot", anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/sensor1"
        msg.min_range = 0
        msg.max_range = 2
        msg.radiation_type = Range.ULTRASOUND

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


class SensorProxy(object):
    '''
    This class subscribes to the zmq messages containing ultrasonic information
    coming from TankoRoboto and sends them over the ROS range topics. Each
    sensor reading gets sent across its own topic.
    '''
    def __init__(self, zmq_address):
        self._sensor_sub, self._ctx = self._setup_zmq(zmq_address)
        self._imu_pub = self._setup_ros()
        signal.signal(signal.SIGINT, self.shutdown)

    def _setup_zmq(self, address):
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, "")
        sock.connect(address)
        return sock, ctx

    def _setup_ros(self):
        rospy.init_node("imu_data", anonymous=True)
        return rospy.Publisher("imu_data", String, queue_size=1)

    def shutdown(self, *args):
        self._sensor_sub.close()
        self._ctx.term()
        rospy.signal_shutdown("We are done now.")

    def run(self):
        while not self._sensor_sub.closed:
            try:
                imu_data = self._sensor_sub.recv()
                self._imu_pub.publish(imu_data)

            except zmq.Again:
                time.sleep(0.01)
            except KeyboardInterrupt:
                print("Interrupted by user, stopping...")
                self.shutdown()
                break
            except Exception as e:
                print("Error: {}".format(e))
                self.shutdown()
                break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-s",
                        "--sensor",
                        help="SensorAddress",
                        required=True)

    parser.add_argument("-u",
                        "--master_uri",
                        help="Address of the ROS master server",
                        required=True)

    args = parser.parse_args()

    os.environ["ROS_MASTER_URI"] = args.master_uri
    proxy = SensorProxy(args.sensor)

    proxy.run()

    print("Received interrupt, shutting down.")


if __name__ == '__main__':
    main()
