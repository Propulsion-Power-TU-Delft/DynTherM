import os
import numpy as np
import matplotlib as mpl
import CoolProp.CoolProp as CP
import matplotlib.pyplot as plt


# user-defined data
X = np.array([-1])       # set to np.array([-1]) for pure fluid
fluid1 = "Water"
fluid2 = "Eglycol"
name = "MIL-PRF-23699"
degree = 8
P = 101325
T_raw = np.arange(223.15, 423.15)

# setup matplotlib to use latex for output
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

mpl.rcParams.update(config)
blues = [plt.get_cmap('Blues_r')(1. * i / (len(X) + 1)) for i in range((len(X) + 1))]
reds = [plt.get_cmap('Reds_r')(1. * i / (len(X) + 1)) for i in range((len(X) + 1))]


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
    if x == -1:
        rho_raw[ii, :] = CP.PropsSI('D', 'T', T_raw, 'P', P, f'REFPROP::{name}')
        c_raw[ii, :] = CP.PropsSI('C', 'T', T_raw, 'P', P, f'REFPROP::{name}')
        kt_raw[ii, :] = CP.PropsSI('CONDUCTIVITY', 'T', T_raw, 'P', P, f'REFPROP::{name}')
        mu_raw[ii, :] = CP.PropsSI('VISCOSITY', 'T', T_raw, 'P', P, f'REFPROP::{name}')
    else:
        rho_raw[ii, :] = CP.PropsSI('D','T', T_raw, 'P', P, f'REFPROP::{fluid1}[{(100 - x) / 100}]&{fluid2}[{x / 100}]')
        c_raw[ii, :] = CP.PropsSI('C','T', T_raw, 'P', P, f'REFPROP::{fluid1}[{(100 - x) / 100}]&{fluid2}[{x / 100}]')
        kt_raw[ii, :] = CP.PropsSI('CONDUCTIVITY', 'T', T_raw, 'P', P, f'INCOMP::{name}-{x}%')
        mu_raw[ii, :] = CP.PropsSI('VISCOSITY', 'T', T_raw, 'P', P, f'INCOMP::{name}-{x}%')

    mask = np.logical_or(np.logical_not(np.isfinite(c_raw[ii, :])), c_raw[ii, :] < 0)
    T[f'{x}%'] = np.delete(T_raw, mask)
    rho[f'{x}%'] = np.delete(rho_raw[ii, :], mask)
    c[f'{x}%'] = np.delete(c_raw[ii, :], mask)

    if x == -1:
        kt[f'{x}%'] = np.delete(kt_raw[ii, :], mask)
        mu[f'{x}%'] = np.delete(mu_raw[ii, :], mask)
    else:
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

    ax1.plot(T[f'{x}%'] - 273.15, rho[f'{x}%'], color=blues[ii], label=f"{name}-{x}\%")
    ax2.plot(T[f'{x}%'] - 273.15, c[f'{x}%'], color=blues[ii], label=f"{name}-{x}\%")
    ax3.plot(T[f'{x}%'] - 273.15, kt[f'{x}%'], color=blues[ii], label=f"{name}-{x}\%")
    ax4.plot(T[f'{x}%'] - 273.15, mu[f'{x}%'], color=blues[ii], label=f"{name}-{x}\%")

    # if x != -1:
    #     ax3.plot(T[f'{x}%'], kt_filtered, 'o')
    #     ax4.plot(T[f'{x}%'], mu_filtered, 'o')

ax1.set_xlabel(r"$T \; [^{\circ}C]$")
ax2.set_xlabel(r"$T \; [^{\circ}C]$")
ax3.set_xlabel(r"$T \; [^{\circ}C]$")
ax4.set_xlabel(r"$T \; [^{\circ}C]$")

ax1.set_ylabel(r"$\rho \; [kg/m^3]$")
ax2.set_ylabel(r"$c_p \; [J/(kg.K)]$")
ax3.set_ylabel(r"$k_t \; [W/(m.K)]$")
ax4.set_ylabel(r"$\mu \; [Pa.s]$")

ax1.grid(True)
ax2.grid(True)
ax3.grid(True)
ax4.grid(True)

# name = "MIL-PRF-23699"
# T_raw = np.arange(223.15, 423.15)
# rho_raw = CP.PropsSI('D', 'T', T_raw, 'P', P, f'REFPROP::{name}')
# c_raw = CP.PropsSI('C', 'T', T_raw, 'P', P, f'REFPROP::{name}')
# kt_raw = CP.PropsSI('CONDUCTIVITY', 'T', T_raw, 'P', P, f'REFPROP::{name}')
# mu_raw = CP.PropsSI('VISCOSITY', 'T', T_raw, 'P', P, f'REFPROP::{name}')
# mask = np.logical_or(np.logical_not(np.isfinite(c_raw)), c_raw < 0)
# T = np.delete(T_raw, mask)
# rho = np.delete(rho_raw, mask)
# c = np.delete(c_raw, mask)
# kt = np.delete(kt_raw, mask)
# mu = np.delete(mu_raw, mask)
# ax1.plot(T - 273.15, rho, color=reds[1], label=f"{name}")
# ax2.plot(T - 273.15, c, color=reds[1], label=f"{name}")
# ax3.plot(T - 273.15, kt, color=reds[1], label=f"{name}")
# ax4.plot(T - 273.15, mu, color=reds[1], label=f"{name}")
#
# ax1.legend()
# ax2.legend()
# ax3.legend()
# ax4.legend()

write_table(X, T, rho, c, kt, mu, name)
fig1.savefig(os.path.join("FluidTableProps", f"{name}_rho.png"), dpi=400)
fig2.savefig(os.path.join("FluidTableProps", f"{name}_c.png"), dpi=400)
fig3.savefig(os.path.join("FluidTableProps", f"{name}_kt.png"), dpi=400)
fig4.savefig(os.path.join("FluidTableProps", f"{name}_mu.png"), dpi=400)
