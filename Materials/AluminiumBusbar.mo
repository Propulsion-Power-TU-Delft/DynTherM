within DynTherM.Materials;
model AluminiumBusbar "Aluminium used for electrical cables"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2700,
    lambda=205,
    cm=900,
    rho0=2.7e-8,
    alpha=0.00429,
    sigma=90e6,
    E=68e9);
end AluminiumBusbar;
