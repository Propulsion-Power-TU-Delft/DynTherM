within DynTherM.Materials;
model AluminiumCable "Aluminium used for electrical cables"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2700,
    lambda=237,
    cm=900,
    kappa=3.69e7,
    alpha_r=0.00429,
    sigma=90e6,
    E=68e9);
end AluminiumCable;
