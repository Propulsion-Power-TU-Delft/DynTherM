within DynTherM.Materials.Paints;
model Aluminium
     extends Modelica.Icons.MaterialProperty;
     extends DynTherM.Materials.Paints.BasePaint(
    eps0=0.2,
    abs0=0,
    greybody=DynTherM.Choices.GreyBodyOpt.Greybody);
end Aluminium;
