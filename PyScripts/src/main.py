# -*- coding: utf-8 -*-

########################################################################################################################
# DynTherM: Dynamic modeling and simulation of Thermal Management systems
# Author: Dr. ir. A. Giuffre'
# Content: main program
# 2024 - Delft University of Technology - All rights reserved
########################################################################################################################

import time
import warnings
from GUI import *


# user-defined input
package_dir = "C:\\Users\\agiuffre\\Projects\\ModelicaProjects\\DynTherM\\package.mo"

if __name__ == '__main__':
    start_time = time.time()
    warnings.filterwarnings("ignore")

    root = tk.Tk()
    root.geometry("300x100+10+10")
    App(root, package_dir).grid()
    root.mainloop()

    print("\n Elapsed time: %10.1f seconds" % (time.time() - start_time))
