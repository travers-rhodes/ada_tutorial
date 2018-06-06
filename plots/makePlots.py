%matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

file_name_templates = ["../%srandom_move_no_camera.txt",
                       "../%srandom_move_with_camera.txt",
                       "../%sconstant_move_no_camera.txt",
                       "../%sconstant_move_with_camera.txt"]

for template in file_name_templates:
  name = template % ""
  dat = np.loadtxt(name, delimiter=",")
  plt.plot(dat[:,0], dat[:,1])

plt.show()