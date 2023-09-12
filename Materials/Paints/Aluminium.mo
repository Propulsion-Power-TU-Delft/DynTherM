within ThermalManagement.Materials.Paints;
model Aluminium
     extends Modelica.Icons.MaterialProperty;
     extends ThermalManagement.Materials.Paints.BasePaint(
       eps0=0.2,
       abs0=0,
       greybody=ThermalManagement.Choices.GreyBodyOpt.Greybody);
end Aluminium;
