# -*- coding: utf-8 -*-

########################################################################################################################
# DynTherM: Dynamic modeling and simulation of Thermal Management systems
# Author: Dr. ir. A. Giuffre'
# Content: input/output
# 2024 - Delft University of Technology - All rights reserved
########################################################################################################################

import os
import configparser
import pandas as pd


def read_config_file(config_file):
    """
    Read configuration file.
    :param config_file:
    :return
        - df_aircraft:
        - df_radiation:
    """
    config = configparser.ConfigParser()
    config.read(os.path.join('..', 'input', config_file + '.txt'))
    n_cases = len(config['ECS']['M_pack'].split()) - 1

    aircraft_data = {
        'Model_dir': [config['ECS']['Model_dir'].split()[i] for i in range(n_cases)],
        'Case_name': [config['ECS']['Case_name'].split()[i] for i in range(n_cases)],
        'Run_name': [config['ECS']['Run_name'].split()[i] for i in range(n_cases)],
        'M_pack': [float(config['ECS']['M_pack'].split()[i]) for i in range(n_cases)],
        'Rec_ratio': [float(config['ECS']['Rec_ratio'].split()[i]) for i in range(n_cases)],
        'T_target': [float(config['ECS']['T_target'].split()[i]) + 273.15 for i in range(n_cases)],
        'T_trim': [float(config['ECS']['T_trim'].split()[i]) + 273.15 for i in range(n_cases)],
        'Mach_inf': [float(config['Environment']['Mach_inf'].split()[i]) for i in range(n_cases)],
        'Height': [float(config['Location']['Height'].split()[i]) * 1e3 for i in range(n_cases)],
        'Delta_ISA': [float(config['Environment']['Delta_ISA'].split()[i]) for i in range(n_cases)],
        'Phi_amb': [float(config['Environment']['Phi_amb'].split()[i]) / 100 for i in range(n_cases)],
        'Phi_ground': [float(config['Environment']['Phi_ground'].split()[i]) / 100 for i in range(n_cases)],
        'T_ground': [float(config['Environment']['T_ground'].split()[i]) + 273.15 for i in range(n_cases)],
        'N_pax': [int(config['Heat_Loads']['N_pax'].split()[i]) for i in range(n_cases)],
        'N_crew': [int(config['Heat_Loads']['N_crew'].split()[i]) for i in range(n_cases)],
        'N_pilots': [int(config['Heat_Loads']['N_pilots'].split()[i]) for i in range(n_cases)],
        'Q_electronics': [float(config['Heat_Loads']['Q_electronics'].split()[i]) for i in range(n_cases)],
        'Q_galley': [float(config['Heat_Loads']['Q_galley'].split()[i]) for i in range(n_cases)],
        'Q_avionics': [float(config['Heat_Loads']['Q_avionics'].split()[i]) for i in range(n_cases)],
        'Cabin_lights': [float(config['Heat_Loads']['Cabin_lights'].split()[i]) for i in range(n_cases)],
        'IFE': [float(config['Heat_Loads']['IFE'].split()[i]) for i in range(n_cases)]
    }

    radiation_data = {
        'Run_name': [config['ECS']['Run_name'].split()[i] for i in range(n_cases)],
        'Latitude': [float(config['Location']['Latitude'].split()[i]) for i in range(n_cases)],
        'Longitude': [float(config['Location']['Longitude'].split()[i]) for i in range(n_cases)],
        'Azimuth': [float(config['Location']['Azimuth'].split()[i]) for i in range(n_cases)],
        'Altitude': [float(config['Location']['Altitude'].split()[i]) for i in range(n_cases)],
        'Height': [float(config['Location']['Height'].split()[i]) for i in range(n_cases)],
        'Year': [int(config['Location']['Year'].split()[i]) for i in range(n_cases)],
        'Month': [int(config['Location']['Month'].split()[i]) for i in range(n_cases)],
        'Day': [int(config['Location']['Day'].split()[i]) for i in range(n_cases)],
        'Hour': [float(config['Location']['Hour'].split()[i]) for i in range(n_cases)],
        'Time_zone': [int(config['Location']['Time_zone'].split()[i]) for i in range(n_cases)],
        'Ref_atmosphere': [config['Location']['Ref_atmosphere'].split()[i] for i in range(n_cases)],
        'Season': [config['Location']['Season'].split()[i] for i in range(n_cases)],
        'Visibility': [float(config['Location']['Visibility'].split()[i]) for i in range(n_cases)],
        'Ground_type': [int(config['Location']['Ground_type'].split()[i]) for i in range(n_cases)],
        'Aerosol': [config['Location']['Aerosol'].split()[i] for i in range(n_cases)]
    }

    df_aircraft = pd.DataFrame(data=aircraft_data)
    df_radiation = pd.DataFrame(data=radiation_data)

    return df_aircraft, df_radiation


def init_output_df():
    """ Initialize dictionary where results will be stored """
    df_res = {
        'Q_tot': [],
        'Q_tot_cab': [],
        'Q_int_cab': [],
        'Q_ext_cab': [],
        'Q_tot_fd': [],
        'Q_int_fd': [],
        'Q_ext_fd': [],
        'T_pack': [],
        'T_fd': [],
        'T_cab': [],
        'T_cargo': [],
        'T_EEbay': [],
        'T_mix': [],
        'Phi_fd': [],
        'Phi_cab': [],
        'Phi_cargo': [],
        'Phi_EEbay': [],
        'M_tot': [],
        'M_fd': [],
        'M_cab': [],
        'M_cargo': [],
        'M_EEbay': [],
        'M_rec': [],
        'M_out': [],
        'FS': [],
        'M_trim_fd': [],
        'M_trim_cab': [],
        'P_pack': [],
        'P_rec': [],
        'P_mix': []
    }
    
    return df_res


def fill_output_df(df_res, model):
    """
    Fill the entries of a pre-initialized dictionary with the results of the simulation
    :param df_res: pre-initialized dictionary used to store results
    :param model: object containing the simulation results
    """
    df_res['Q_tot'].append(model.Q[0])
    df_res['Q_tot_cab'].append(model.Q[1])
    df_res['Q_int_cab'].append(model.Q[2])
    df_res['Q_ext_cab'].append(model.Q[3])
    df_res['Q_tot_fd'].append(model.Q[4])
    df_res['Q_int_fd'].append(model.Q[5])
    df_res['Q_ext_fd'].append(model.Q[6])
    df_res['T_pack'].append(model.T[0] - 273.15)
    df_res['T_fd'].append(model.T[1] - 273.15)
    df_res['T_cab'].append(model.T[2] - 273.15)
    df_res['T_cargo'].append(model.T[3] - 273.15)
    df_res['T_EEbay'].append(model.T[4] - 273.15)
    df_res['T_mix'].append(model.T[5] - 273.15)
    df_res['Phi_fd'].append(model.phi[0] * 100)
    df_res['Phi_cab'].append(model.phi[1] * 100)
    df_res['Phi_cargo'].append(model.phi[2] * 100)
    df_res['Phi_EEbay'].append(model.phi[3] * 100)
    df_res['M_tot'].append(- model.mass[0])
    df_res['M_fd'].append(model.mass[1])
    df_res['M_cab'].append(model.mass[2])
    df_res['M_cargo'].append(model.mass[3])
    df_res['M_EEbay'].append(model.mass[4])
    df_res['M_rec'].append(model.mass[5])
    df_res['M_out'].append(model.mass[6])
    df_res['FS'].append(model.mass[7])
    df_res['M_trim_fd'].append(- model.mass[8])
    df_res['M_trim_cab'].append(- model.mass[9])
    df_res['P_pack'].append(model.P[0] / 1e3)
    df_res['P_rec'].append(model.P[-2] / 1e3)
    df_res['P_mix'].append(model.P[-1] / 1e3)
    
    return df_res
