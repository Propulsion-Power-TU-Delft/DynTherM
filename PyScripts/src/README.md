# Tmp
ciao

---

### Description
ciao

---

### How to get started
ciao

---

### Pre-requisites
##### [Python 3.7+](https://python.org)
##### [NumPy](https://numpy.org)
##### [MatPlotLib](https://matplotlib.org)

---

### Input file structure
**ECS**

* M_pack [kg/s]:
* Rec_ratio [-]:
* Xw_pack [-]:
* T_target [°C]:
* M_trim_cab [kg/s]:
* M_trim_fd [kg/s]:
* T_trim [°C]:

**Environment**

* Mach_inf [-]
* Delta_ISA [°C]
* Phi_amb [%]
* Phi_ground [%]:
* T_ground [°C]:

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

* N_pax [-]:
* N_crew [-]:
* N_pilots [-]:
* Q_electronics [W]:
* Q_galley [W]:
* Q_avionics [W]:
* Cabin_lights [%]:
* IFE [%]: