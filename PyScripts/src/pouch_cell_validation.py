# -*- coding: utf-8 -*-

########################################################################################################################
# DynTherM: Dynamic modeling and simulation of Thermal Management systems
# Author: Dr. ir. A. Giuffre'
# Content: validation of pouch cell model
# 2024 - Delft University of Technology - All rights reserved
########################################################################################################################

import os
import DyMat
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
from dymola.dymola_interface import DymolaInterface


# user-defined input
package_dir = os.path.join('C:\\Users', 'Public', 'Projects', 'ModelicaProjects', 'DynTherM', 'package.mo')
df = pd.read_excel(open(os.path.join('..', 'input', 'pouch_cell_validation.xlsx'), 'rb'), sheet_name='Ark1')

config = {
    "pgf.texsystem": "pdflatex",
    "text.usetex": True,
    "font.family": "DejaVu Sans",
    "axes.titlesize": 24,
    "axes.labelsize": 24,
    "font.size": 24,
    "legend.fontsize": 18,
    "legend.frameon": True,
    "xtick.labelsize": 18,
    "ytick.labelsize": 18,
    "xtick.major.pad": 8,
    "ytick.major.pad": 8,
    "figure.autolayout": True,
    "figure.figsize": (9.6, 7.2)
}

# mpl.rcParams.update(config)

if __name__ == '__main__':
    # Instantiate the Dymola interface and start Dymola
    dymola = DymolaInterface()
    dymola.openModel(path=package_dir)

    # run the prescribed model in Dymola
    dymola.ExecuteCommand("Evaluate = false")
    dymola.ExecuteCommand("Advanced.EvaluateAlsoTop = false")
    dymola.ExecuteCommand("Advanced.Translation.SmartSimulateExtended = true")
    result = dymola.simulateExtendedModel("DynTherM.Validation.Battery.PouchCellPolestar", 0.0, 1200.0, 0, 1.0,
                                          "Dassl", 0.0001, 0.0, "Simulations/validation_pouch_cell", [], [], [],
                                          True)
    if result[0]:
        # read results file
        results = DyMat.DyMatFile(os.path.join('..', '..', 'Simulations', 'validation_pouch_cell.mat'))
        t = results.mat['data_2'][0]
        T_cv = np.vstack((DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[1].T_vol'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[2].T_vol'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[3].T_vol'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[4].T_vol'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[5].T_vol'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[6].T_vol'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[7].T_vol'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[8].T_vol'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[9].T_vol'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[10].T_vol'))) - 273.15
        Q_cv = np.vstack((DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[1].Q_gen'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[2].Q_gen'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[3].Q_gen'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[4].Q_gen'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[5].Q_gen'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[6].Q_gen'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[7].Q_gen'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[8].Q_gen'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[9].Q_gen'),
                          DyMat.DyMatFile.data(results, 'pouchCell1D.thermal.cv[10].Q_gen')))

    else:
        print("\nDymola simulation did not converge")
        log = dymola.getLastErrorLog()
        print(log)

    fig1, ax1 = plt.subplots(figsize=(9.6, 8.0))
    ax1.plot(df['Time_T [s]'], df['T_min [C]'], color='blue', linestyle='dashed')
    ax1.plot(df['Time_T [s]'], df['T_avg [C]'], color='black', linestyle='dashed')
    ax1.plot(df['Time_T [s]'], df['T_max [C]'], color='red', linestyle='dashed')
    ax1.plot(t, T_cv[0, :], color='blue', linestyle='solid', label='min')
    ax1.plot(t, T_cv[4, :], color='black', linestyle='solid', label='avg')
    ax1.plot(t, T_cv[9, :], color='red', linestyle='solid', label='max')
    ax1.legend()
    # ax1.set_xlabel(r'$t \; \mathrm[s]$')
    # ax1.set_ylabel(r'$T \; [^{\circ}C]$')
    ax1.grid(1)

    fig2, ax2 = plt.subplots(figsize=(9.6, 8.0))
    ax2.plot(df['Time_Q [s]'], df['Q_gen [W]'], color='black', linestyle='dashed', label='reference')
    ax2.plot(t, Q_cv[0, :] * 10, color='black', linestyle='solid', label='model')
    ax2.legend()
    ax2.set_xlabel('t [s]')
    ax2.set_ylabel('Q [W]')
    # ax2.set_xlabel(r'$t \; \mathrm[s]$')
    # ax2.set_ylabel(r'$\dot{Q} \; \mathrm{[W]}$')
    ax2.grid(1)

    fig1.savefig(os.path.join('..', 'output', 'T_polestar.png'), dpi=400)
    fig2.savefig(os.path.join('..', 'output', 'Q_polestar.png'), dpi=400)
