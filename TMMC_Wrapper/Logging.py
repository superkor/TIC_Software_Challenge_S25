import time
import subprocess
import os

class Logging:
    @staticmethod
    def configure_logging(robot,topics):
        robot.logging_topics = topics
        
    @staticmethod    
    def start_logging(robot):
        if hasattr(robot,'logging_instance'):
            raise Exception("logging already active")
        robot.logging_dir = bag_dir = '/tmp/notebook_bag_'+str(int(time.time()))
        robot.logging_instance = subprocess.Popen("ros2 bag record -s mcap --output "+robot.logging_dir+" "+' '.join(robot.logging_topics)+" > /tmp/ros2_bag.log 2>&1",shell=True,stdout=subprocess.PIPE,preexec_fn=os.setsid)
        # Wait until topics are subscribed
        # TODO: check log for this
        time.sleep(5)
        
    @staticmethod
    def stop_logging(robot):
        import signal
        os.killpg(os.getpgid(robot.logging_instance.pid), signal.SIGINT)
        robot.logging_instance.wait()
        del robot.logging_instance
        return robot.logging_dir
           
    @staticmethod 
    def get_logging_data(logging_dir):
        # get log data
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=logging_dir,storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('','')
        reader.open(storage_options,converter_options)
        from rosidl_runtime_py.utilities import get_message
        import rosbag2_py
        from rclpy.serialization import deserialize_message
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

    @staticmethod
    def delete_logging_data(logging_dir):
        import shutil
        shutil.rmtree(logging_dir)
