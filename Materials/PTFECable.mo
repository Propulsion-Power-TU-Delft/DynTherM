within DynTherM.Materials;
model PTFECable "PTFE used as electrical insulator"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2170,
    lambda=0.25,
    cm=1500,
    epsilon=2.1,
    sigma=24e6,
    E=0.49e9);
end PTFECable;
