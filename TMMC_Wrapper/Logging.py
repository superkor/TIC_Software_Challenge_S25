import time
import subprocess
import os
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import shutil
import signal

class Logging:
    def __init__(self, robot):
        self.robot = robot

    def configure_logging(self, topics):
        self.robot.logging_topics = topics
          
    def start_logging(self):
        if hasattr(self.robot,'logging_instance'):
            raise Exception("logging already active")
        self.robot.logging_dir = bag_dir = '/tmp/notebook_bag_'+str(int(time.time()))
        self.robot.logging_instance = subprocess.Popen("ros2 bag record -s mcap --output "+self.robot.logging_dir+" "+' '.join(self.robot.logging_topics)+" > /tmp/ros2_bag.log 2>&1",shell=True,stdout=subprocess.PIPE,preexec_fn=os.setsid)
        time.sleep(5)
        
    def stop_logging(self):
        os.killpg(os.getpgid(self.robot.logging_instance.pid), signal.SIGINT)
        self.robot.logging_instance.wait()
        del self.robot.logging_instance
        return self.robot.logging_dir
            
    def get_logging_data(self, logging_dir):
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=logging_dir,storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('','')
        reader.open(storage_options,converter_options)
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        log_content = dict()
        while reader.has_next():
            (topic,data,t) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            if topic not in log_content.keys():
                log_content[topic] = []
            log_content[topic].append((t,msg))
        return log_content

    def delete_logging_data(self, logging_dir):
        shutil.rmtree(logging_dir)
