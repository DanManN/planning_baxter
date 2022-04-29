from get_poses import Perception
import sys
import os
import time
import datetime
import numpy as np
import math

sys.path.append("..")
from actionsToCommands import actionToCommands


class error_analysis(object):
    def __init__(self, pool='R') -> None:
        self.P = Perception(pool=pool)
        self.letter = list(pool)[0]
        self.logs = logs()

    def perception_analysis(self):
        for x in [-10.0, 0.0, 10.0]:
            for y in [-10.0, 0.0]:
                goal_pose = (x,y,0.0,self.letter)
                # goal_pose = (0.0,0.0,0.0,self.letter)
                print("goal pose", goal_pose)
                self.repetition(goal_pose)
                self.perception(goal_pose)

    def repetition(self, goal_pose):
        # repetition
        starts = []
        results = []
        for trial in range(3):
            print("repetition trial: ", trial)
            start_pose = self.get_current_pose()
            starts.append(start_pose)
            self.move_to_goal(start_pose, goal_pose)
            self.P.wait_for_execution()
            result = self.get_current_pose()
            results.append(result)
            if trial == 0:
                self.logs.write_to_transformation_error(self.letter, start_pose, goal_pose, result)
            self.logs.write_to_repetition_log(self.letter, trial, start_pose, goal_pose, result)

    def perception(self, goal_pose):
        # perception
        pose_list = []
        for trial in range(10):
            print("perception trial: ", trial)
            predicted_pose = self.get_current_pose()
            pose_list.append(predicted_pose)
        self.logs.write_to_perception_error(self.letter, goal_pose, pose_list)

    def translation_analysis(self):
        goal_list = [
            (x1, y1, 0.0, self.letter) for x1 in [-10.0, 0.0, 10.0] for y1 in [-10.0, 0.0]
            ]
        for _ in range(10):
            goalID = np.random.choice(range(len(goal_list)))
            goal = goal_list[goalID]
            print('current goal', goal)
            start_pose = self.get_current_pose()
            self.move_to_goal(start_pose, goal)
            self.P.wait_for_execution()
            result = self.get_current_pose()
            self.logs.write_to_translation_error(self.letter, start_pose, goal, result)
            self.repetition(goal)

        
    def rotation_analysis(self):
        theta_list = [i*math.pi/4.0 for i in range(8)]
        goal_pose = (0.0,0.0,0.0, self.letter)
        self.repetition(goal_pose)
        for x in [-10.0, 0.0, 10.0]:
            for y in [-10.0, 0.0, 10.0]:
                for i in range(10):
                    theta = np.random.choice(theta_list)
                    goal_pose = (x,y,theta,self.letter)
                    print('current goal', goal_pose)
                    start_pose = self.get_current_pose()
                    self.move_to_goal(start_pose, goal_pose)
                    self.P.wait_for_execution()
                    result = self.get_current_pose()
                    if i == 0:
                        self.logs.write_to_transformation_error(self.letter, start_pose, goal_pose, result)
                    else:
                        self.logs.write_to_rotation_error(self.letter, start_pose, goal_pose, result)
                    self.repetition(goal_pose)
                    
    def camera_error(self):
        for x in [-10.0, 0.0, 10.0]:
            for y in [-10.0, 0.0, 10.0]:
                for _ in range(10):
                    # back to origin
                    goal_pose = (0.0, 0.0, 0.0, self.letter)
                    self.repetition(goal_pose)
                    goal_pose = (x,y,0.0,self.letter)
                    start_pose = self.get_current_pose()
                    self.move_to_goal(start_pose, goal_pose)
                    self.P.wait_for_execution()
                    result = self.get_current_pose()
                    self.logs.write_to_translation_error(
                        self.letter, start_pose, goal_pose, result
                        )
                    
        
                




    def move_to_goal(self, start_pose, goal_pose):
        action = (0, start_pose, goal_pose)
        actionToCommands([action])

    def get_current_pose(self):
        arr_list = self.P.get_arrangement(purpose="S")
        return arr_list[0]


class logs(object):
    def __init__(self) -> None:
        super().__init__()
        # generate an unique experiment ID
        timestamp = time.time()
        timestamp_value = datetime.datetime.fromtimestamp(timestamp)
        self.experimentID = timestamp_value.strftime("%Y-%m-%d-%H-%M-%S")

        log_dir = repetition_log_file = os.path.join(
            os.path.dirname(
                os.path.abspath(__file__)), "logs", self.experimentID
        )
        os.makedirs(log_dir)
        # Initialize logs
        # repetition log
        self.repetition_log_file = os.path.join(
            log_dir, 'repetition_log.txt')
        with open(self.repetition_log_file, 'w') as f:
            f.write('letter, goal, trial, error, start-goal-diff\n')
        # perception
        self.perception_log_file = os.path.join(
            log_dir, 'perception_log.txt')
        with open(self.perception_log_file, 'w') as f:
            f.write('letter, goal, predicted-poses\n')

        # translation
        self.translation_log_file = os.path.join(
            log_dir, 'translation_log.txt')
        with open(self.translation_log_file, 'w') as f:
            f.write('letter, goal, error, start-goal-diff\n')
        
        # rotation
        self.rotation_log_file = os.path.join(
            log_dir, 'rotation_log.txt')
        with open(self.rotation_log_file, 'w') as f:
            f.write('letter, goal, error, start-goal-diff\n')
        
        # transformation
        self.transformation_log_file = os.path.join(
            log_dir, 'transformation_log.txt')
        with open(self.transformation_log_file, 'w') as f:
            f.write('letter, goal, error, start-goal-diff\n')


    def write_to_repetition_log(self, letter, trial, start, goal, result):
        error = tuple(
                np.array(result[:3]) - np.array(goal[:3])
            )
        diff = tuple(
                np.array(goal[:3]) - np.array(start[:3])
            )
        with open(self.repetition_log_file, 'a') as f:
            f.write(letter + str(goal)+ ' ' +str(trial)+' '+ str(error)+ ' ' +str(diff) + '\n')


    def write_to_perception_error(self, letter, goal, pose_list):
        error_list = [
            tuple(np.array(p[:3])-np.array(goal[:3])) for p in pose_list
        ]
        with open(self.perception_log_file, 'a') as f:
            f.write((letter + ' ' + str(goal) + ' ' + str(error_list) + '\n'))

    def write_to_translation_error(self, letter,  start, goal, result):
        error = tuple(
                np.array(result[:3]) - np.array(goal[:3])
            )
        diff = tuple(
                np.array(goal[:3]) - np.array(start[:3])
            )
        with open(self.translation_log_file, 'a') as f:
            f.write(letter + str(goal)+ ' ' + str(error)+ ' ' +str(diff) + '\n')

    def write_to_rotation_error(self, letter,  start, goal, result):
        error = tuple(
                np.array(result[:3]) - np.array(goal[:3])
            )
        diff = tuple(
                np.array(goal[:3]) - np.array(start[:3])
            )
        with open(self.rotation_log_file, 'a') as f:
            f.write(letter + str(goal)+ ' ' + str(error)+ ' ' +str(diff) + '\n')

    def write_to_transformation_error(self, letter,  start, goal, result):
        error = tuple(
                np.array(result[:3]) - np.array(goal[:3])
            )
        diff = tuple(
                np.array(goal[:3]) - np.array(start[:3])
            )
        with open(self.transformation_log_file, 'a') as f:
            f.write(letter + str(goal)+ ' ' + str(error)+ ' ' +str(diff) + '\n')


if __name__ == "__main__":
    EA = error_analysis(pool='R')
    # EA.perception_analysis()
    EA.translation_analysis()
    # EA.rotation_analysis()
    # EA.camera_error()
    # EA.repetition((0.0, 0.0, 0.0, 'R'))
