within DynTherM.Materials.Paints;
model BasePaint
  constant Real eps0 "Fuselage paint emissivity";
  constant Real abs0 "Fuselage paint absorptivity" annotation (Dialog(enable = not greybody));
  parameter DynTherM.Choices.GreyBodyOpt greybody=DynTherM.Choices.GreyBodyOpt.notGreybody;
  Real abs;
  Real eps;
equation
  if greybody == DynTherM.Choices.GreyBodyOpt.notGreybody then
    abs = abs0;
    eps = eps0;
  elseif greybody == DynTherM.Choices.GreyBodyOpt.Greybody then
    abs = eps0;
    eps = eps0;
  end if;
end BasePaint;
