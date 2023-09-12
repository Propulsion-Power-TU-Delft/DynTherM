within DynTherM.Materials.Paints;
model GreyOilPaint "Grey oil paint used for fuselage exteriors"
     extends Modelica.Icons.MaterialProperty;
     extends DynTherM.Materials.Paints.BasePaint(
    eps0=0.9,
    abs0=0.75,
    greybody=DynTherM.Choices.GreyBodyOpt.notGreybody);
end GreyOilPaint;
