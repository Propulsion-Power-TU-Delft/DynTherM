# -*- coding: utf-8 -*-

########################################################################################################################
# DynTherM: Dynamic modeling and simulation of Thermal Management systems
# Author: Dr. ir. A. Giuffre'
# Content: graphical user interface
# 2024 - Delft University of Technology - All rights reserved
########################################################################################################################

import pyfiglet
import tkinter as tk
from src.IO import *
from tqdm import tqdm
from src.radiation import SMARTS
from src.aircraft import Aircraft


class App(tk.Frame):
    """
    Class to take user input from a GUI and run the main program
    """
    def __init__(self, root, package_dir, master=None, **kw):
        tk.Frame.__init__(self, master=master, **kw)
        idx = 0
        self.settings = {}
        self.root = root
        self.font = ('calibre', 10)
        self.package_dir = package_dir
        self.res_file = "init"

        self.root.title('Welcome to DynTherM!')

        print("\n# ----------------------------------------------------------------------------------------------- #")
        banner = pyfiglet.figlet_format("DynTherM")
        print(banner)
        print("  Dynamic modeling and simulation of Thermal Management systems")
        print("  Author: Dr. ir. A. Giuffre'")
        print("  Delft University of Technology - All rights reserved\n")
        print("# ----------------------------------------------------------------------------------------------- #\n\n")

        tk.Label(self, text="Configuration file", justify='left', font=self.font).grid(row=idx, column=0)
        self.q1 = tk.Entry(self, font=self.font)
        self.q1.grid(row=idx, column=1)
        idx += 1

        tk.Label(self, text="Result file", justify='left', font=self.font).grid(row=idx, column=0)
        self.q2 = tk.Entry(self, font=self.font)
        self.q2.grid(row=idx, column=1)
        idx += 1

        tk.Button(self, text="Run", command=self.run, justify='left', font=self.font).grid(row=idx, column=1)

    def run(self):
        """
        Run the main program and save results in an Excel file
        """
        # read input file
        df_aircraft, df_radiation = read_config_file(self.q1.get())
        n_cases = df_aircraft.shape[0]

        # initialize dataframe to store results
        df_res = init_output_df()
        
        self.res_file = self.q2.get()
        self.root.destroy()

        # create Simulations folder (if it does not exist already)
        if not os.path.isdir(os.path.join('..', '..', 'Simulations')):
            os.makedirs(os.path.join('..', '..', 'Simulations'))

        for idx in tqdm(range(n_cases)):
            solar_radiation = SMARTS()
            solar_radiation.run('aircraft', df_radiation, idx)
            model = Aircraft(self.package_dir)
            model.run(df_aircraft, solar_radiation, idx)
            df_res = fill_output_df(df_res, model)

        with pd.ExcelWriter(os.path.join('..', 'output', self.res_file + '.xlsx')) as writer:
            df_res = pd.DataFrame(data=df_res)
            df_res = df_res.T
            df_res.to_excel(writer)
