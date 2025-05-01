within DynTherM.Materials;
model XLPECable "XLPE used as electrical insulator"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=930,
    lambda=0.29,
    cm=2174,
    epsilon=2.3,
    sigma=18e6,
    E=0.6e9);
end XLPECable;
