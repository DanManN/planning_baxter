#!/usr/bin/env python
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

directory = "experiments"
trial = int(sys.argv[1])

with open(f"{directory}/exp{2*trial:04d}.csv") as f:
    objs = [l.split(',')[0:3:2] for l in f.readlines()[1:]]
    objs.sort(key=lambda x: x[1])
    names = [' '.join(x[0].split('_')[1:-1]) for x in objs]

f, a = plt.subplots(2, 1)
img = mpimg.imread(f"{directory}/exp{2*trial:04d}.jpg")
a[0].imshow(img)
a[0].set_title("Start")
a[0].xaxis.set_ticks([160, 320, 480])
a[0].xaxis.set_ticklabels(names)
a[0].yaxis.set_ticks([])
img = mpimg.imread(f"{directory}/exp{2*trial+1:04d}.jpg")
a[1].imshow(img)
a[1].set_title("Goal")
a[1].xaxis.set_ticks([160, 320, 480])
a[1].xaxis.set_ticklabels(names)
a[1].yaxis.set_ticks([])
plt.show(block=False)

task_description = input(
    """
Please type directions for the robot to take the objects from their initial orientations to their goal orientations.
(Please use the provided object names in your description.)
  \n"""
)

if task_description:
    with open(f"{directory}/exp{trial:04d}.txt", "w") as f:
        print(task_description, file=f)
