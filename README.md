# DynTherM

---

### Description

A Modelica library for the Dynamic modeling and simulation of Thermal Management systems.

Applications:

1. Aircraft fuselage

2. Helicopter fuselage (to be adapted and ported to the new version of DynTherM)

3. Hydrogen tank (coming soon)

4. Battery pack (coming soon)

![aircraft_model](./Figures/aircraft_model.pdf)

---

### Author
* **A. Giuffré**, Post Doctoral Researcher, Propulsion & Power, TU Delft

[![Link MailTo](https://img.shields.io/badge/MailTo-developers-blue.svg)](mailto:a.giuffre@tudelft.nl;c.m.deservi@tudelft.nl?subject=DynTherM:Query)

### Acknowledgements
* **C. De Servi**, Senior Researcher, Propulsion & Power, TU Delft - Main Technical Advisor
* **I. Gul**, PhD Researcher, Propulsion & Power, TU Delft - Battery Pack
* **M. Swart**, MSc Student, TU Delft - Hydrogen Tank
* **K. Guimaraes**, MSc Student, TU Delft - Helicopter Fuselage

---

### Pre-requisites
##### [Modelica 4.0](https://www.modelica.org/)
##### [Dymola 2022](https://www.3ds.com/products-services/catia/products/dymola/) (not tested with OpenModelica)
##### [ExternalMedia](https://github.com/modelica-3rdparty/ExternalMedia)
##### [Python 3.9](https://python.org) (optional, only for Python - Dymola interface)
##### [NumPy](https://numpy.org) (optional, only for Python - Dymola interface)
```
sudo pip install numpy
```
##### [MatPlotLib](https://matplotlib.org) (optional, only for Python - Dymola interface)
```
sudo pip install matplotlib
```
##### [pandas](https://pandas.pydata.org/) (optional, only for Python - Dymola interface)
```
sudo pip install pandas
```
##### [tqdm](https://pypi.org/project/tqdm/) (optional, only for Python - Dymola interface)
```
sudo pip install tqdm
```
##### [SMARTS 2.9.5](https://www.nrel.gov/grid/solar-resource/smarts.html) (optional, only for Python - Dymola interface)

---

### Citations
A. Giuffré, P. Colonna, and C. De Servi. "Dynamic Thermal Model of Passenger Aircraft for the Estimation of the Cabin Cooling and Heating Requirements", Applied Thermal Engineering, 2024.

---

### Setting Python - Dymola interface

The following instructions are tested for Python 3.7+ and Dymola 2022, assuming the Dymola installation folder to be the standard one.

1. Add to Path (environment variable): C:\\Program Files\\Dymola 2022\\bin64\\Dymola.exe

2. Create PYTHONPATH (environment variable), if not existing yet, and add: C:\Program Files\Dymola 2022\Modelica\Library\python_interface\dymola.egg

3. Set package_dir in main.py as "path-to-package.mo (inside DynTherM)" according to your system directory; set model_dir accordingly

4. Download SMARTS from NREL website and install it in the DynTherM root folder

    Upon completion of the previous steps, you should be able to run main.py from command line or python IDLE (remember to open the Dymola app before running main.py).
    The next step is only required to run main.py from PyCharm.

5. Open PyCharm and go to File/Settings/Project Interpreter. At the top right, select show all, show paths for the selected interpreter and add: C:\Program Files\Dymola 2022\Modelica\Library\python_interface/dymola.egg
    
---

### How to get started
1. Load package.mo in Dymola
  
2. Open and run one of the models in package Examples.mo

3. Analyze the results

