import os
import numpy as np
import CoolProp.CoolProp as CP
import matplotlib.pyplot as plt


# user-defined data
X = np.arange(10, 60, 10)       # set to -1 for pure fluid
fluid1 = "Water"
fluid2 = "Eglycol"
name = "MEG"
degree = 8
P = 101325
T_raw = np.arange(223.15, 393.15)


def write_table(X, T, rho, c, kt, mu, name):
    """
    Write thermo-physical fluid properties in a tabular format compatible with Modelica

    Parameters
    ---
    :param X: array of mixture fractions (if applicable)
    :param T: dictionary containing temperature data entries
    :param rho: dictionary containing density data entries
    :param c: dictionary containing specific heat capacity data entries
    :param kt: dictionary containing thermal conductivity data entries
    :param mu: dictionary containing dynamic viscosity data entries
    :param name: name of the table
    """
    if not os.path.isdir("FluidTableProps"):
        os.makedirs("FluidTableProps")

    out_file = open(os.path.join("FluidTableProps", f'{name}.txt'), "w+")

    out_file.write("// ------------------------------------- TABULATED DENSITY -------------------------------------\n")
    for x in X:
        if x == -1:
            out_file.write(f"  constant Real[{len(T[f'{x}%'])},2] tableDensity = [")
        else:
            out_file.write(f"  constant Real[{len(T[f'{x}%'])},2] tableDensity_{x} = [")

        for jj in range(len(T[f'{x}%'])):
            if jj == 0:
                out_file.write(f"{T[f'{x}%'][jj]:.2f}, {rho[f'{x}%'][jj]:.4f};\n")
            elif jj == (len(T[f'{x}%']) - 1):
                out_file.write(f"                                          "
                               f"{T[f'{x}%'][jj]:.2f}, {rho[f'{x}%'][jj]:.4f}];\n\n")
            else:
                out_file.write(f"                                          "
                               f"{T[f'{x}%'][jj]:.2f}, {rho[f'{x}%'][jj]:.4f};\n")

    out_file.write("// ------------------------------ TABULATED SPECIFIC HEAT CAPACITY -----------------------------\n")
    for x in X:
        if x == -1:
            out_file.write(f"  constant Real[{len(T[f'{x}%'])},2] tableHeatCapacity = [")
        else:
            out_file.write(f"  constant Real[{len(T[f'{x}%'])},2] tableHeatCapacity_{x} = [")

        for jj in range(len(T[f'{x}%'])):
            if jj == 0:
                out_file.write(f"{T[f'{x}%'][jj]:.2f}, {c[f'{x}%'][jj]:.4f};\n")
            elif jj == (len(T[f'{x}%']) - 1):
                out_file.write(f"                                               "
                               f"{T[f'{x}%'][jj]:.2f}, {c[f'{x}%'][jj]:.4f}];\n\n")
            else:
                out_file.write(f"                                               "
                               f"{T[f'{x}%'][jj]:.2f}, {c[f'{x}%'][jj]:.4f};\n")

    out_file.write("// ------------------------------- TABULATED THERMAL CONDUCTIVITY ------------------------------\n")
    for x in X:
        if x == -1:
            out_file.write(f"  constant Real[{len(T[f'{x}%'])},2] tableConductivity = [")
        else:
            out_file.write(f"  constant Real[{len(T[f'{x}%'])},2] tableConductivity_{x} = [")

        for jj in range(len(T[f'{x}%'])):
            if jj == 0:
                out_file.write(f"{T[f'{x}%'][jj]:.2f}, {kt[f'{x}%'][jj]:.8f};\n")
            elif jj == (len(T[f'{x}%']) - 1):
                out_file.write(f"                                               "
                               f"{T[f'{x}%'][jj]:.2f}, {kt[f'{x}%'][jj]:.8f}];\n\n")
            else:
                out_file.write(f"                                               "
                               f"{T[f'{x}%'][jj]:.2f}, {kt[f'{x}%'][jj]:.8f};\n")

    out_file.write("// --------------------------------- TABULATED DYNAMIC VISCOSITY -------------------------------\n")
    for x in X:
        if x == -1:
            out_file.write(f"  constant Real[{len(T[f'{x}%'])},2] tableViscosity = [")
        else:
            out_file.write(f"  constant Real[{len(T[f'{x}%'])},2] tableViscosity_{x} = [")

        for jj in range(len(T[f'{x}%'])):
            if jj == 0:
                out_file.write(f"{T[f'{x}%'][jj]:.2f}, {mu[f'{x}%'][jj]:.8f};\n")
            elif jj == (len(T[f'{x}%']) - 1):
                out_file.write(f"                                            "
                               f"{T[f'{x}%'][jj]:.2f}, {mu[f'{x}%'][jj]:.8f}];\n\n")
            else:
                out_file.write(f"                                            "
                               f"{T[f'{x}%'][jj]:.2f}, {mu[f'{x}%'][jj]:.8f};\n")

    out_file.close()
    
    
# initialization
samples = len(T_raw)
rho_raw = np.zeros((len(X), samples))
c_raw = np.zeros((len(X), samples))
kt_raw = np.zeros((len(X), samples))
mu_raw = np.zeros((len(X), samples))
T = {}
rho = {}
c = {}
kt = {}
mu = {}

# compute thermo-physical properties
fig1, ax1 = plt.subplots()
fig2, ax2 = plt.subplots()
fig3, ax3 = plt.subplots()
fig4, ax4 = plt.subplots()

for ii, x in enumerate(X):
    rho_raw[ii, :] = CP.PropsSI('D','T', T_raw, 'P', P, f'REFPROP::{fluid1}[{(100 - x) / 100}]&{fluid2}[{x / 100}]')
    c_raw[ii, :] = CP.PropsSI('C','T', T_raw, 'P', P, f'REFPROP::{fluid1}[{(100 - x) / 100}]&{fluid2}[{x / 100}]')
    kt_raw[ii, :] = CP.PropsSI('CONDUCTIVITY', 'T', T_raw, 'P', P, f'INCOMP::{name}-{x}%')
    mu_raw[ii, :] = CP.PropsSI('VISCOSITY', 'T', T_raw, 'P', P, f'INCOMP::{name}-{x}%')

    mask = np.logical_or(np.logical_not(np.isfinite(c_raw[ii, :])), c_raw[ii, :] < 0)
    T[f'{x}%'] = np.delete(T_raw, mask)
    rho[f'{x}%'] = np.delete(rho_raw[ii, :], mask)
    c[f'{x}%'] = np.delete(c_raw[ii, :], mask)
    kt_filtered = np.delete(kt_raw[ii, :], mask)
    mu_filtered = np.delete(mu_raw[ii, :], mask)

    mask = np.isfinite(kt_filtered)
    kt_coeff = np.polyfit(T[f'{x}%'][mask], kt_filtered[mask], degree)
    kt_fit = np.poly1d(kt_coeff)
    kt[f'{x}%'] = kt_fit(T[f'{x}%'])

    mask = np.isfinite(mu_filtered)
    mu_coeff = np.polyfit(T[f'{x}%'][mask], mu_filtered[mask], degree)
    mu_fit = np.poly1d(mu_coeff)
    mu[f'{x}%'] = mu_fit(T[f'{x}%'])

    ax1.plot(T[f'{x}%'], rho[f'{x}%'])
    ax2.plot(T[f'{x}%'], c[f'{x}%'])
    ax3.plot(T[f'{x}%'], kt[f'{x}%'])
    ax3.plot(T[f'{x}%'], kt_filtered, 'o')
    ax4.plot(T[f'{x}%'], mu[f'{x}%'])
    ax4.plot(T[f'{x}%'], mu_filtered, 'o')

write_table(X, T, rho, c, kt, mu, name)
fig1.savefig(os.path.join("FluidTableProps", f"{name}_rho.png"), dpi=400)
fig2.savefig(os.path.join("FluidTableProps", f"{name}_c.png"), dpi=400)
fig3.savefig(os.path.join("FluidTableProps", f"{name}_kt.png"), dpi=400)
fig4.savefig(os.path.join("FluidTableProps", f"{name}_mu.png"), dpi=400)
