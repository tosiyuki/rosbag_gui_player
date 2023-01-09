#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import threading
import subprocess

import yaml
import rospy


class Player:
    def __init__(self):
        self.bag = None
        self.start_time = 0
        self.end_time = 0
        self.duration = 0
        self.play_speed = 1.0

        self.stop_flag = False
        self.publisers ={}
        self.topic_msgs = {}
        self.threads = []

        self._now_time = 0
        self._stop_time = 0
        self._timer_thread_start_time = 0

    @property
    def now_time(self):
        return self._now_time

    @now_time.setter
    def now_time(self, now_time):
        self._now_time = now_time
        self._stop_time = now_time
        self._timer_thread_start_time = time.time() - self._now_time

    def create_publisher(self):
        if self.bag is None:
            return

        self.publisers.clear()
        for topic, msg, timestamp in self.bag.read_messages():
            if topic not in self.publisers:
                self.publisers[topic] = rospy.Publisher(topic, type(msg), queue_size=100)
                self.topic_msgs[topic] = [[msg, timestamp]]
            else:
                self.topic_msgs[topic].append([msg, timestamp])
        
    def play(self, topics, start_time_reset=False):
        if self.bag is None:
            return

        if start_time_reset:
            self._now_time = 0
            self._stop_time = 0

        self.stop_flag = False
        self.threads.clear()

        for topic in topics:
            self.threads.append(threading.Thread(target=self._publish_msg, args=(topic,)))

        for thread in self.threads:
            thread.start()

        self.timer_thread = threading.Thread(target=self._play_timer)
        self.timer_thread.start()

    def stop(self):
        self.stop_flag = True
        self._stop_time = self._now_time

        # wait finished all threads
        for thread in self.threads:
            thread.join()
        self.timer_thread.join()

    def read_rosbag_info(self, rosbag_path):
        # reference : 1.7 http://wiki.ros.org/rosbag/Cookbook
        info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', rosbag_path], stdout=subprocess.PIPE).communicate()[0])
        self.start_time = info_dict['start']
        self.end_time = info_dict['end']
        self.duration = info_dict['duration']

    def _publish_msg(self, topic_name):
        for topic_msg in self.topic_msgs[topic_name]:
            msg = topic_msg[0]
            time_stamp = topic_msg[1].to_sec()
            while True:
                if (time_stamp - self.start_time) < self._stop_time:
                    # skip message before start time
                    break
                
                if (time_stamp - self.start_time) < self._now_time:
                    # publish
                    self.publisers[topic_name].publish(msg)
                    break

                if self.stop_flag:
                    # end
                    return

                time.sleep(0.00001)
            
    def _play_timer(self):
        self._timer_thread_start_time = time.time() - (self._now_time / self.play_speed)
        
        while True:
            self._now_time = time.time() - self._timer_thread_start_time
            self._now_time *= self.play_speed
            
            if self.stop_flag:
                # end
                return
            elif self._now_time > self.duration:
                # finish playing
                self.stop_flag = True
                return

            time.sleep(0.001)
