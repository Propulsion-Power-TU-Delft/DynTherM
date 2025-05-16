within DynTherM.Materials;
model Properties
  constant Density rho "Material density";
  constant ThermalConductivity lambda "Thermal conductivity";
  constant SpecificHeatCapacity cm "Specific heat capacity";
  constant Resistivity rho0 "Resistivity at 20 degrees Celsius";
  constant Permittivity epsilon "Permittivity";
  constant CustomUnits.ResistanceTemperatureScaling alpha "Resistance-temperature scale factor";
  constant Stress sigma "Tensile strength";
  constant Stress E "Modulus of elasticity";
  constant Real n "Refractive index";
  constant Real t "Transmittance";
end Properties;
