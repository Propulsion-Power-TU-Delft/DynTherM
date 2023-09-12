within DynTherM.Materials.Paints;
model ExteriorGreyMetal
  extends Modelica.Icons.MaterialProperty;
  extends DynTherM.Materials.Paints.BasePaint(
    eps0=0.92,
    abs0=0,
    greybody=DynTherM.Choices.GreyBodyOpt.Greybody);
end ExteriorGreyMetal;
