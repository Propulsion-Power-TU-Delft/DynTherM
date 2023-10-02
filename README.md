# DynTherM

---

### Description

A Modelica library for the Dynamic simulation of Thermal Management systems.

---

### Author
* **A. Giuffré**, Post Doctoral Researcher, Propulsion & Power, TU Delft

### Acknowledgements
* **C. De Servi**, Assistant Professor, Propulsion & Power, TU Delft
* **K. Guimaraes**, MSc student, TU Delft

---

### Pre-requisites
##### [Modelica 4.0](https://www.modelica.org/)
##### [Dymola 2022](https://www.3ds.com/products-services/catia/products/dymola/) (not tested with OpenModelica)
##### [Python 3.9](https://python.org)
##### [NumPy](https://numpy.org)
```
sudo pip install numpy
```
##### [MatPlotLib](https://matplotlib.org)
```
sudo pip install matplotlib
```
##### [pandas](https://pandas.pydata.org/)
```
sudo pip install pandas
```
##### [tqdm](https://pypi.org/project/tqdm/)
```
sudo pip install tqdm
```
##### [SMARTS 2.9.5](https://www.nrel.gov/grid/solar-resource/smarts.html) (optional, only for Python - Dymola interface)

---

### Citations

---

### Setting Python - Dymola interface

Disclaimer: the Python scripts are not released open-source yet, since they still need to be cleaned and extensively tested.

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

