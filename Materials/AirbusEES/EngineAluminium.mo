within DynTherM.Materials.AirbusEES;
model EngineAluminium "NH90: Engine wall aluminium"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho = 2.700,
    lambda = 0.1,
    cm = 900);
end EngineAluminium;
