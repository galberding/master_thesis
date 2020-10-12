import imageio
import os
import time

IMAGES = "../../../devel/lib/ros_optimizer/res"
OUT = "../../../devel/lib/ros_optimizer/train_{}.gif".format(time.strftime("%Y%m%d-%H%M%S"))

filenames = []
for root, dirs, files in os.walk(IMAGES, topdown=False):
    print(files)
    for f in files:
        filenames.append(os.path.join(IMAGES,f))

        images = []
for filename in filenames:
    images.append(imageio.imread(filename))
imageio.mimsave(OUT, images)
