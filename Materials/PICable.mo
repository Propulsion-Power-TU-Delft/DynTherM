within DynTherM.Materials;
model PICable "PI used as electrical insulator"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=1380,
    lambda=0.4,
    cm=1090,
    epsilon=3.5,
    sigma=96e6,
    E=3.1e9);
end PICable;
