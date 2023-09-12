within DynTherM.Materials.AirbusEES;
model TransmissionAluminium "NH90: Transmission wall aluminium"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho = 2.700,
    lambda = 0.1,
    cm = 900);
end TransmissionAluminium;
