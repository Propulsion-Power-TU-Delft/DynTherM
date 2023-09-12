within DynTherM.Materials.Paints;
model WhiteOilPaint "White oil paint used for fuselage exteriors"
     extends Modelica.Icons.MaterialProperty;
     extends DynTherM.Materials.Paints.BasePaint(
    eps0=0.9,
    abs0=0.3,
    greybody=DynTherM.Choices.GreyBodyOpt.notGreybody);
end WhiteOilPaint;
