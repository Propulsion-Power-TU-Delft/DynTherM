within DynTherM.Materials;
model CopperCable "Copper used for electrical cables"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=8900,
    lambda=400,
    cm=385,
    kappa=5.87e7,
    alpha_r=0.00394,
    sigma=210e6,
    E=110e9);
end CopperCable;
