#!/usr/bin/env python3
import rospy
import json
import os
import autonomous_navigation

class ConfigurationFiles():
    def __init__(self, directory) -> None:
            self.directory = directory
            self.json_data_list = self.read_all_json_files_in_directory(self.directory)

    def __read_json_file(self, filename):
        try:
            with open(filename, 'r') as file:
                data = json.load(file)
            return data
        except FileNotFoundError:
            print(f"File '{filename}' not found.")
            return {}

    def read_all_json_files_in_directory(self, directory):
        json_data_list = []
        for filename in sorted(os.listdir(directory)):
            if filename.endswith('.json'):
                full_path = os.path.join(directory, filename)
                json_data = self.__read_json_file(full_path)

                if json_data:
                    json_data_list.append(json_data)
        return json_data_list

class CCU():
    def __init__(self):
        self.auto_nav = autonomous_navigation.navigation_on_map()
        directory = '/home/touhid/catkin_ws/src/tc200_hda/src/user_side_scripts/shelf_list'  # Change this to the directory containing your JSON files
        myConfig = ConfigurationFiles(directory)
        self.json_data_list = myConfig.read_all_json_files_in_directory(directory)

    def run_dummy(self):
        # read jobs
        dummy_jobs = self.read_jobs(None)
        # dummy_jobs = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        dummy_jobs = [0, 1, 2]

        for job in dummy_jobs:
            # auto nav to job blocking
            successfull = self.auto_nav.move2pose(-0.4663,-2.0768, -2.8801)

            # perform job
            if successfull:
                self.set_job_description(None)
                print("Start with job: ", job)
                self.start_job(None)
                while rospy.get_param("/bath/enabled_functionality") == 1:
                    rospy.sleep(0.01)
                print("Done with job: ", job)
            else:
                print("skip job, shelf could not be reached")
                self.auto_nav.cancel()
                

    def run(self):
        if self.json_data_list:
            for idx, json_data in enumerate(self.json_data_list, start=1):

                self.shelf_start_point_x = json_data["shelf_start_point_x"]
                self.shelf_start_point_y = json_data["shelf_start_point_y"]
                self.shelf_start_point_theta = json_data["shelf_start_point_theta"]
                successfull = self.auto_nav.move2pose(self.shelf_start_point_x,self.shelf_start_point_y, self.shelf_start_point_theta)

                if successfull:
                    self.shelf_start_direction = json_data["direction"]
                    self.shelf_distance = json_data["distance"]
                    if self.shelf_start_direction == 'left':
                        rospy.set_param("/bath/linear_vel_y", 0.06) # Towards left
                    elif self.shelf_start_direction == 'right':
                        rospy.set_param("/bath/linear_vel_y", -0.06) # Towards right
                    rospy.set_param("/bath/distance_2_wall", 1.5)
                    rospy.set_param("/bath/shelf_length", self.shelf_distance)

                    print("Start with File: ", idx)
                    self.start_job(None)
                    while rospy.get_param("/bath/enabled_functionality") == 1:
                        rospy.sleep(0.01)
                    print("Done with File: ", idx)
                else:
                    print("skip file, shelf could not be reached")
                    self.auto_nav.cancel()
                
                rospy.sleep(1)
        else:
            print("No JSON data available.")


    def start_job(self, config):
        rospy.set_param("/bath/enabled_functionality", 1)


if __name__ == '__main__':
    rospy.init_node("UseCase")
    CCU().run()