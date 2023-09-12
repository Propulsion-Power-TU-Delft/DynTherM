within ThermalManagement.Materials.Paints;
model WhiteOilPaint "White oil paint used for fuselage exteriors"
     extends Modelica.Icons.MaterialProperty;
     extends ThermalManagement.Materials.Paints.BasePaint(
       eps0=0.9,
       abs0=0.3,
       greybody=ThermalManagement.Choices.GreyBodyOpt.notGreybody);
end WhiteOilPaint;
