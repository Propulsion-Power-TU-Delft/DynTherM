within ThermalManagement.Materials.AirbusEES;
model Window "NH90: Window"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho = 2.500,
    lambda = 0.25,
    cm = 840,
    n = 1.52,
    t = 0.9);
end Window;
