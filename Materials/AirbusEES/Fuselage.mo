within DynTherM.Materials.AirbusEES;
model Fuselage "NH90: Fuselage material"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho = 2.700,
    lambda = 0.1,
    cm = 9);
end Fuselage;
