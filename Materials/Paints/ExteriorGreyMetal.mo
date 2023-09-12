within ThermalManagement.Materials.Paints;
model ExteriorGreyMetal
  extends Modelica.Icons.MaterialProperty;
  extends ThermalManagement.Materials.Paints.BasePaint(
    eps0 = 0.92,
    abs0 = 0,
    greybody = ThermalManagement.Choices.GreyBodyOpt.Greybody);
end ExteriorGreyMetal;
