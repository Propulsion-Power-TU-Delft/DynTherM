within ThermalManagement.Materials.Paints;
model WhitePaint "White paint used for fuselage exteriors"
     extends Modelica.Icons.MaterialProperty;
     extends ThermalManagement.Materials.Paints.BasePaint(
       eps0=0.9,
       abs0=0.2,
       greybody=ThermalManagement.Choices.GreyBodyOpt.notGreybody);
end WhitePaint;
