within DynTherM.Materials;
model PFACable "PFA used as electrical insulator"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2150,
    lambda=0.19,
    cm=1047,
    epsilon=2.1,
    sigma=28e6,
    E=0.55e9);
end PFACable;
