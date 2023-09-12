import os
import time
import warnings
import subprocess
import pandas as pd
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import HeliScenarios as HS 

# TODO: ADD EXTERNAL SELECTION OF CONVECTION MODEL
# TODO: ADD CORRECTION OF T_GROUND WHILE FLYING
# TODO: ADD LOOP OVER TE ST CASES
# TODO: ADD SMARTS INFO TO DATAFRAME BEING WRITTEN
# TODO: ADD CHECK TO CONFIRM TGT TEMP AND ACTUAL TEMP MATCH

# data for calculation of solar radiation
run_name = 'Airbus Test'
tilt_angle = np.array([70, 110, 250, 290])    # [deg] from horizontal 
# latitude = 33.753746        # [deg] positive north
# longitude = -84.386330      # [deg] positive east
azimuth = 240.0             # [deg] clockwise from north of RHS of vehicle
# altitude = 0.3              # site altitude [km]
# height = 0.0                # object height from ground [km]
year = 2020                 # four digits
month = 7                   # [1 - 12]
day = 20                    # [1 - 31]
hour = 12                   # [0.0 - 24.0]
time_zone = -5              # UTC/GMT
# ref_atmosphere = 'USSA'     # reference atmospheric conditions: check SMARTS user manual card 3a
season = 'SUMMER'           # choose 'WINTER' for fall and 'SUMMER' for spring
visibility = 500            # prevailing visibility observed at airports [0.77 - 764 km]
#                             parameter necessary at ground conditions, not during flight above 6 km
aerosol = 'S&F_TROPO'       # aerosol model: check SMARTS user manual card 8
ground_type = 35            # select an index [3 - 66] corresponding to a ground type, e.g. 35 --> sea water
#                             check SMARTS user manual card 10

class SMARTS:
    def __init__(self, name, lat, long, az, alt, height, y, m, d, h, tz, tilt, vis, atm, aer, ground,
                 folder='C:\\Users\\kikow\\Dropbox\\Thesis - Kiko Guimaraes\\ModelicaCodeLocalChanges\\thermalmanagement\\SMARTS_2.9.5', input_file='smarts295.inp.txt', output_file='smarts295.out.txt'):
        self.run_name = name
        self.latitude = lat
        self.longitude = long
        self.azimuth = az
        self.tilt_angle = tilt
        self.altitude = alt
        self.height = height
        self.year = y
        self.month = m
        self.day = d
        self.hour = h
        self.time_zone = tz
        self.ref_atmosphere = atm
        self.aerosol = aer
        self.ground_type = ground
        self.folder = folder
        self.input_file = input_file
        self.output_file = output_file
        self.phi_amb = np.array([])
        self.theta = np.array([])
        self.E_direct = np.array([])
        self.E_diffuse = np.array([])
        self.E_reflected = np.array([])
        self.E = np.array([])
        if self.height < 6:
            if vis is None:
                raise ValueError("Visibility must be specified if height < 6 km")
            else:
                self.visibility = vis
        else:
            self.tau550 = np.exp(-3.2755 - 0.15078 * (self.altitude + self.height))

    def isSubstring(self, s1, s2):
        """ Returns true if s1 is substring of s2 """
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

    def writeInputFile(self, tilt=None, azimuth=None):
        """ Write input file in the format required by SMARTS 2.9.5 """
        if tilt is None:
            tilt = self.tilt_angle
        if azimuth is None:
            azimuth = self.azimuth

        out_file = open(self.folder + '/' + self.input_file, "w+")
        out_file.write("%s" % self.run_name + "\n") # C1 Run Name
        out_file.write("2   \n") #C2 ISPR
        out_file.write("%3.4f  %3.4f  %3.4f" % (self.latitude, self.altitude, self.height) + "\n") #C2a 
        out_file.write("1   \n") #C3 IATMOS
        out_file.write("%s" % self.ref_atmosphere + "\n") #C3a
        out_file.write("1   \n") #C4 IH2O
        out_file.write("1   \n") #C5 IO3
        out_file.write("1   \n") #C6 IGAS
        out_file.write("370.0   \n") #C7 qCO2
        out_file.write("0   \n") #C7a 
        out_file.write("%s" % self.aerosol + "\n") #C8
        if self.height < 6:
            out_file.write("4   \n") #C9 ITURB
            out_file.write("%3.4f" % self.visibility + "\n") #C9a VISI
        else:
            out_file.write("5   \n") #C9 ITURB
            out_file.write("%3.4f" % self.tau550 + "\n") #C9a TAU550
        out_file.write("%2d" % self.ground_type + "\n") #C10 IALBDX
        out_file.write("1   \n") #C10b ITILT
        out_file.write("%2d  %3.4f  %3.4f" % (self.ground_type, tilt, azimuth) + "\n") #C10c
        out_file.write("280  4000  1.0  1367.0  \n") #C11
        out_file.write("0   \n") #C12 IPRT
        out_file.write("0   \n") #C13 ICIRC
        out_file.write("0   \n") #C14 ISCAN
        out_file.write("0   \n") #C15 ILLUM
        out_file.write("0   \n") #C16 IUV
        out_file.write("3   \n") #C17 IMASS
        out_file.write("%4d  %2d  %2d  %2.2f  %3.4f  %3.4f  %2d"
                       % (self.year, self.month, self.day, self.hour, self.latitude, self.longitude, self.time_zone) + "\n") #C17a
        out_file.close()

        return

    def readOutputFile(self):
        """ Read quantities of interest from the output file of SMARTS 2.9.5 """
        with open(self.folder + '/' + self.output_file, "r") as f:
            lines = f.readlines()
            for ii, line in enumerate(lines):
                if self.isSubstring('INPUTS', line) != -1:
                    self.phi_amb = np.append(self.phi_amb, float(lines[ii + 3].split()[4]) / 100)
                elif self.isSubstring('ANGLES (deg.) FOR TILTED SURFACE CALCULATIONS', line) != -1:
                    self.theta = np.append(self.theta, np.deg2rad(float(lines[ii + 2].split()[3])))
                elif self.isSubstring('FOR THE TILTED PLANE', line) != -1:
                    self.E_direct = np.append(self.E_direct, float(lines[ii + 1].split()[3]))
                    self.E_diffuse = np.append(self.E_diffuse, float(lines[ii + 1].split()[7]))
                    self.E_reflected = np.append(self.E_reflected, float(lines[ii + 1].split()[11]))

            f.close()

        return

    def cleanAfterRun(self):
        """ Delete input and output files after running SMARTS 2.9.5 """
        try:
            os.remove(self.folder + '/' + self.input_file)
            os.remove(self.folder + '/' + self.output_file)
        except:
            warnings.warn('Root directory is already clean; use this method only after running SMARTS 2.9.5')

        return

    def run(self):
        """ Open a new cmd and run the batch executable of SMARTS 2.9.5 """
        # compute solar radiation in the different sections composing the cylindrical part of the fuselage
        for tilt in self.tilt_angle:
            self.writeInputFile(tilt=tilt)
            p = subprocess.Popen('start smarts295bat', cwd=self.folder, shell=True)
            time.sleep(0.5)
            p.terminate()
            self.readOutputFile()
            self.cleanAfterRun()

        self.E = self.E_direct + self.E_diffuse + self.E_reflected
        return

### Run Simulation
excel_file = 'HeliTestCases.xlsx'
model_name = "AtlantaGround"
locations = {'Atl_Gro': {'City': 'Atlanta, US', 'Lat': 33.753746, 'Long': -84.386330, 'rAtmos': 'USSA', 'Alt': 0.3, 'Hgt': 0}, \
                'Atl_2km': {'City': 'Atlanta, US', 'Lat': 33.753746, 'Long': -84.386330, 'rAtmos': 'USSA', 'Alt': 0.3, 'Hgt': 2}, \
                'Atl_5km': {'City': 'Atlanta, US', 'Lat': 33.753746, 'Long': -84.386330, 'rAtmos': 'USSA', 'Alt': 0.3, 'Hgt': 5}, \
                'Man_Gro': {'City': 'Manaus, BR', 'Lat': -3.132485, 'Long': -60.01825, 'rAtmos': 'TRL', 'Alt': 0.01, 'Hgt': 0}, \
                'Man_2km': {'City': 'Manaus, BR', 'Lat': -3.132485, 'Long': -60.01825, 'rAtmos': 'TRL', 'Alt': 0.01, 'Hgt': 2}, \
                'Man_5km': {'City': 'Manaus, BR', 'Lat': -3.132485, 'Long': -60.01825, 'rAtmos': 'TRL', 'Alt': 0.01, 'Hgt': 5}, \
            }

# Test Conditions to run [T_ck, T_cb, Alt=0, ISA, Npax]
test_conditions = np.array([[30, 30, 0, 30, 20], 
                            [28, 28, 0, 30, 20],
                            [25, 25, 0, 30, 20],
                            [30, 30, 0, 25, 20], 
                            [28, 28, 0, 25, 20],
                            [25, 25, 0, 25, 20],
                            [30, 30, 0, 20, 20], 
                            [28, 28, 0, 20, 20],
                            [25, 25, 0, 20, 20],
                            ])
results_list = []
case_idx = 0
for location in locations:
    if location == 'Atl_Gro':
        solarRadiation = SMARTS(location, locations[location]['Lat'], locations[location]['Long'], azimuth, locations[location]['Alt'], locations[location]['Hgt'], year, 
                        month, day, hour, time_zone, tilt_angle, visibility, locations[location]['rAtmos'], aerosol, ground_type)
        solarRadiation.run()
        for i in range(np.size(test_conditions,0)):
            test_conditions[i][2] = (locations[location]['Alt'] + locations[location]['Hgt'])*1000 # Correct the altitude for environment
            model = HS.HeliCombined(test_conditions[i].tolist(), solarRadiation, case_idx)
            model.run()
            results_dict = model.writeResultsDict()
            results_list.append(results_dict)
            case_idx += 1

# Write to Excel
df = pd.DataFrame(results_list)
df.set_index("Case Number", inplace=True)
with pd.ExcelWriter(excel_file, mode='a') as writer:
    df.to_excel(writer, sheet_name=model_name)

# Plot data 
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot_trisurf(df['ISA'], df['Cb T_tgt'], df['Cb Q'], linewidth=0.2)
ax.set_xlabel("ISA+ [C]")
ax.set_ylabel("Cb T_tgt [C]")
ax.set_zlabel("Evap Q [W]")
ax.set_title("Cabin Evap Q - Atlanta Ground")
plt.show()





