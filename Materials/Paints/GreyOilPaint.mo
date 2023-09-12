within ThermalManagement.Materials.Paints;
model GreyOilPaint "Grey oil paint used for fuselage exteriors"
     extends Modelica.Icons.MaterialProperty;
     extends ThermalManagement.Materials.Paints.BasePaint(
       eps0=0.9,
       abs0=0.75,
       greybody=ThermalManagement.Choices.GreyBodyOpt.notGreybody);
end GreyOilPaint;
