# DynTherM
Dynamic modeling and simulation of Thermal Management systems 

---

### Description
Python wrapper used to automate the simulation of multiple aircraft test cases

---

### How to get started
1. Open main.py and specify package_dir, i.e., the absolute path pointing to the DynTherM Modelica library on your machine.
2. Copy and edit one of the sample configuration files located in the input folder.
3. Run main.py and fill the two entries in the graphical user interface. 
By default, the configuration file needs to be located in the input folder, 
and the output Excel file will be generated in the output folder.
4. Click the run button and wait until the simulation(s) is over.
5. Open the result file in the output folder and analyze the results.

---

### Input file structure
One configuration file can contain the definition of an arbitrary number of test cases.
The entries related to different test cases are separated by space tabs.
In the models named like A320GroundCabin the temperature at the discharge of the ECS pack is automatically regulated
to meet the target temperature in the cabin, whereas the fine-tuning of the temperature in the cockpit is achieved by 
adjusting the opening of the trim air valve. The opposite is true for the models named like A320GroundCockpit.

**ECS**

* Model_dir: package where the model is stored under DynTherM.Examples.Aircraft
* Case_name: name of the model to run
* Run_name: name associated to the simulation
* M_pack [kg/s]: mass flow rate provided by the two ECS packs combined
* Rec_ratio [-]: ratio between the recirculated and the total mass flow rate
* T_target [°C]: target temperature in the prescribed ventilation zone
* M_trim_cab [kg/s]: mass flow rate of trim air provided to the cabin
* M_trim_fd [kg/s]: mass flow rate of trim air provided to the flight deck
* T_trim [°C]: temperature of the trim air

**Environment**

* Mach_inf [-]: free-stream Mach number
* Delta_ISA [°C]: temperature deviation with respect to ISA conditions
* Phi_amb [%]: ambient relative humidity
* Phi_ground [%]: ambient relative humidity on ground
* T_ground [°C]: temperature of the ground surface, e.g., airport runaway

**Location**

* Latitude[deg]: site’s latitude, measured positive North, negative South.
* Longitude [deg]: site's longitude, measured positive East of Greenwich, negative West of Greenwich.
* Azimuth [deg]: surface azimuth, measured clockwise from North.
* Altitude [km]: site’s altitude, i.e., elevation of the ground surface above sea level; 
it must be ≤ 100 km. In case of a flying object, it refers to the ground surface below it.
* Height [km]: height of the simulated object above the ground surface underneath; it must be ≤ 100 km.
* Year [-]: a four-digit integer, e.g., 2002.
* Month [-]: any integer between 1 and 12.
* Day [-]: any integer between 1 and 31.
* Hour [-]: Local Standard Time (LST) in decimal hour.
* Time_zone [-]: International Time Zone relative to Greenwich, with the same sign convention as longitude.
* Ref_atmosphere [-]: string identifying the reference atmospheric conditions; check SMARTS user manual card 3a.
* Season [-]: string identifying the season. If the true season is Fall, select WINTER. Select SUMMER if the true season is Spring.
* Visibility [km]: Prevailing visibility, as observed at airports; it must be between 0.77 and 764.
It is required at ground conditions, not during flight above 6 km.
* Ground_type [-]: integer between 3 and 66, corresponding to a ground type, 
e.g. 35 --> sea water, 18 --> concrete. Check SMARTS user manual card 10.
* Aerosol [-]: string identifying the aerosol model, check SMARTS user manual card 8.

**Heat_Loads**

* N_pax [-]: number of passengers inside the cabin
* N_crew [-]: number of crew members inside the cabin
* N_pilots [-]: number of pilots inside the cockpit
* Q_electronics [W]: heat load related to the flight deck electronics
* Q_galley [W]: heat load related to the galleys inside the cabin
* Q_avionics [W]: heat load related to the avionics bay located below the cockpit
* Cabin_lights [%]: percentage of utilization of the cabin lights
* IFE [%]: percentage of utilization of the in-flight entertainment