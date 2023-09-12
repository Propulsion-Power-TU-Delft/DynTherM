within ThermalManagement.Materials;
model CarbonPhenolic "Carbon phenolic"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=1800,
    lambda=1,
    cm=600);
end CarbonPhenolic;
