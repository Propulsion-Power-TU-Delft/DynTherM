within ThermalManagement.Materials;
model Aluminium "Aluminium used for aircraft skin"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2710,
    lambda=140,
    cm=900);
end Aluminium;
