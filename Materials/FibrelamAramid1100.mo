within DynTherM.Materials;
model FibrelamAramid1100
  "Fibrelam® 1100 Aramid phenolic honeycomb HRH-10-1/8-4.0"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=64,
    lambda=0.0675,
    cm=1300);
end FibrelamAramid1100;
