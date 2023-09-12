within DynTherM.Materials.AirbusEES;
model Floor "NH90: Floor material"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho = 2.700,
    lambda = 1,
    cm = 900);
end Floor;
