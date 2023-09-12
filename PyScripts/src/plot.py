import numpy as np
import matplotlib.pyplot as plt


def polar_contour(values, cbar_label,
                  azimuth_min=0, azimuth_max=360, azimuth_secs=8,
                  zenith_min=0, zenith_max=1, zenith_secs=2,
                  cmap='hot'):

    if values.ndim == 1:
        zenith_secs = 2
        values = np.column_stack((values, values))

    azimuth = np.radians(np.linspace(azimuth_min, azimuth_max, azimuth_secs))
    zenith = np.linspace(zenith_min, zenith_max, zenith_secs)
    r, theta = np.meshgrid(zenith, azimuth)

    fig, ax = plt.subplots(subplot_kw=dict(projection='polar'))
    CS = ax.contourf(theta, r, values, cmap=cmap)
    cbar = fig.colorbar(CS, shrink=1.0)
    cbar.ax.set_xlabel(cbar_label)

    return


def scatter_plot(values, labels, y_label, cmap='hot'):

    fig, ax = plt.subplots()
    new_colors = [plt.get_cmap(cmap)(1. * i / len(values)) for i in range(len(values))]

    for ii in range(len(values)):
        ax.plot(ii, values[ii], marker='o', color=new_colors[ii], label=labels[ii])

    ax.set_ylabel(y_label)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels)
    ax.grid(1)

    return
