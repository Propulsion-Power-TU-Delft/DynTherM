within ThermalManagement.Materials.AirbusEES;
model Duct "NH90: Duct material"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho = 2.700,
    lambda = 0.05,
    cm = 900);
end Duct;
