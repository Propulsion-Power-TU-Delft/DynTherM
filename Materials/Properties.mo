within DynTherM.Materials;
model Properties
  constant Density rho "Material density";
  constant ThermalConductivity lambda "Thermal conductivity";
  constant SpecificHeatCapacity cm "Specific heat capacity";
  constant Conductivity kappa "Electrical conductivity";
  constant Permittivity epsilon "Permittivity";
  constant CustomUnits.ResistanceTemperatureScaling alpha_r "Resistance-temperature scale factor";
  constant Stress sigma "Tensile strength";
  constant Stress E "Modulus of elasticity";
  constant Real n "Refractive index";
  constant Real t "Transmittance";
end Properties;
