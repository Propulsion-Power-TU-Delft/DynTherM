# -*- coding: utf-8 -*-

########################################################################################################################
# ThermalManagement
# Author: ir. A. Giuffre'
# Content: main program
# 2022 - TU Delft - All rights reserved
########################################################################################################################

import warnings
from src.IO import *
from tqdm import tqdm
from src.radiation import SMARTS
from src.aircraft import Aircraft


# user-defined input
package_dir = "C:\\Users\\agiuffre\\Projects\\ModelicaProjects\\thermalmanagement\\package.mo"
cfg_file = "config_validation"
res_file = "validation"

# ------------------------------------------------------------------------------------------------------------------- #

# read input file
df_aircraft, df_radiation = read_config_file(cfg_file)
n_cases = df_aircraft.shape[0]

# initialize dataframe to store results
df_res = init_output_df()

warnings.filterwarnings("ignore")
print("\n********************************************************")
print("  ThermalManagement - Aircraft Heat Loads Estimation")
print("  Author: ir. A. Giuffre'")
print("  Delft University of Technology - All rights reserved")
print("********************************************************")


for idx in tqdm(range(n_cases)):
    solarRadiation = SMARTS()
    solarRadiation.run('aircraft', df_radiation, idx)
    model = Aircraft(package_dir)
    model.run(df_aircraft, solarRadiation, idx)
    df_res = fill_output_df(df_res, model)

with pd.ExcelWriter(os.path.join('..', 'output', res_file + '.xlsx')) as writer:
    df_res = pd.DataFrame(data=df_res)
    df_res = df_res.T
    df_res.to_excel(writer)
