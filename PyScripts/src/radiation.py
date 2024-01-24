# -*- coding: utf-8 -*-

########################################################################################################################
# DynTherM: Dynamic modeling and simulation of Thermal Management systems
# Author: Dr. ir. A. Giuffre'
# Content: solar radiation computation with SMARTS
# 2024 - Delft University of Technology - All rights reserved
########################################################################################################################

import os
import time
import subprocess
import numpy as np


class SMARTS:
    """
    Class to perform solar radiation calculations with SMARTS 2.9.5
    """
    def __init__(self, input_file='smarts295.inp.txt', output_file='smarts295.out.txt'):
        """
        :param input_file:
        :param output_file:
        """
        self.input_dir = os.path.join('..', '..', 'SMARTS_2.9.5', input_file)
        self.output_dir = os.path.join('..', '..', 'SMARTS_2.9.5', output_file)

        self.phi_amb = np.array([])
        self.theta = np.array([])
        self.E_direct = np.array([])
        self.E_diffuse = np.array([])
        self.E_reflected = np.array([])
        self.E = np.array([])

    @staticmethod
    def is_substring(s1, s2):
        """
        Returns true if s1 is substring of s2
        :param s1: substring to be searched
        :param s2: string where the search is performed
        """
        m = len(s1)
        n = len(s2)
        jj = 0

        for ii in range(n - m + 1):
            for jj in range(m):
                if s2[ii + jj] != s1[jj]:
                    break
            if (jj + 1) == m:
                return ii

        return -1

    def write_input_file(self, radiation_data, idx, tilt, azimuth):
        """
        Write input file in the format required by SMARTS 2.9.5
        :param radiation_data:
        :param idx:
        :param tilt:
        :param azimuth:
        """
        out_file = open(self.input_dir, "w+")
        out_file.write("%s  \n" % radiation_data["Run_name"][idx])
        out_file.write("2   \n")
        out_file.write("%3.4f  %3.4f  %3.4f" % (radiation_data["Latitude"][idx], radiation_data["Altitude"][idx],
                                                radiation_data["Height"][idx]) + "\n")
        out_file.write("1   \n")
        out_file.write("%s" % radiation_data["Ref_atmosphere"][idx] + "\n")
        out_file.write("1   \n")
        out_file.write("1   \n")
        out_file.write("1   \n")
        out_file.write("370.0   \n")
        out_file.write("0   \n")
        out_file.write("%s" % radiation_data["Aerosol"][idx] + "\n")

        if radiation_data["Height"][idx] < 6:
            out_file.write("4   \n")
            out_file.write("%3.4f" % radiation_data["Visibility"][idx] + "\n")
        else:
            out_file.write("5   \n")
            tau550 = np.exp(-3.2755 - 0.15078 * (radiation_data["Altitude"][idx] + radiation_data["Height"][idx]))
            out_file.write("%3.4f" % tau550 + "\n")

        out_file.write("%2d" % radiation_data["Ground_type"][idx] + "\n")
        out_file.write("1   \n")
        out_file.write("%2d  %3.4f  %3.4f" % (radiation_data["Ground_type"][idx], tilt, azimuth) + "\n")
        out_file.write("280  4000  1.0  1367.0  \n")
        out_file.write("0   \n")
        out_file.write("0   \n")
        out_file.write("0   \n")
        out_file.write("0   \n")
        out_file.write("0   \n")
        out_file.write("3   \n")
        out_file.write("%4d  %2d  %2d  %2.2f  %3.4f  %3.4f  %2d"
                       % (radiation_data["Year"][idx], radiation_data["Month"][idx], radiation_data["Day"][idx],
                          radiation_data["Hour"][idx], radiation_data["Latitude"][idx],
                          radiation_data["Longitude"][idx], radiation_data["Time_zone"][idx]) + "\n")
        out_file.close()

        return

    def read_output_file(self):
        """ Read quantities of interest from the output file of SMARTS 2.9.5 """
        with open(self.output_dir, "r") as f:
            lines = f.readlines()
            for ii, line in enumerate(lines):
                if self.is_substring('INPUTS', line) != -1:
                    self.phi_amb = np.append(self.phi_amb, float(lines[ii + 3].split()[4]) / 100)

                elif self.is_substring('ANGLES (deg.) FOR TILTED SURFACE CALCULATIONS', line) != -1:
                    self.theta = np.append(self.theta, np.deg2rad(float(lines[ii + 2].split()[3])))

                elif self.is_substring('FOR THE TILTED PLANE', line) != -1:
                    self.E_direct = np.append(self.E_direct, float(lines[ii + 1].split()[3]))
                    self.E_diffuse = np.append(self.E_diffuse, float(lines[ii + 1].split()[7]))
                    self.E_reflected = np.append(self.E_reflected, float(lines[ii + 1].split()[11]))

            f.close()

        return

    def clean_after_run(self):
        """ Delete input and output files after running SMARTS 2.9.5 """
        os.remove(self.input_dir)
        os.remove(self.output_dir)

        return

    def run(self, application, radiation_data, idx):
        """
        Open a new cmd and run the batch executable of SMARTS 2.9.5
        :param application: string identifying the type of application under analysis
        :param radiation_data:
        :param idx:
        """
        if application == 'aircraft':
            # compute solar radiation over the eight sections composing the cylindrical part of the fuselage
            tilt_angles = np.array([270.0, 315.0, 0.0, 45.0, 90.0, 135.0, 180.0, 225.0])

            for tilt in tilt_angles:
                self.write_input_file(radiation_data, idx, tilt, radiation_data["Azimuth"][idx])
                p = subprocess.Popen('start smarts295bat', cwd='../../SMARTS_2.9.5', shell=True)
                time.sleep(0.5)
                p.terminate()
                self.read_output_file()
                self.clean_after_run()

            # compute solar radiation in the frontal part of the cockpit: windshield of the flight deck
            self.write_input_file(radiation_data, idx, 45, radiation_data["Azimuth"][idx] + 90)
            p = subprocess.Popen('start smarts295bat', cwd='../../SMARTS_2.9.5', shell=True)
            time.sleep(0.5)
            p.terminate()
            self.read_output_file()
            self.clean_after_run()

            self.E = self.E_direct + self.E_diffuse + self.E_reflected

        else:
            raise NotImplementedError("Currently the only available application is 'aircraft'")

        return
