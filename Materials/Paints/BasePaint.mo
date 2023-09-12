within ThermalManagement.Materials.Paints;
model BasePaint
  constant Real eps0 "Fuselage paint emissivity";
  constant Real abs0 "Fuselage paint absorptivity" annotation (Dialog(enable = not greybody));
  parameter ThermalManagement.Choices.GreyBodyOpt greybody=ThermalManagement.Choices.GreyBodyOpt.notGreybody;
  Real abs;
  Real eps;
equation
  if greybody == ThermalManagement.Choices.GreyBodyOpt.notGreybody then
    abs = abs0;
    eps = eps0;
  elseif greybody == ThermalManagement.Choices.GreyBodyOpt.Greybody then
    abs = eps0;
    eps = eps0;
  end if;
end BasePaint;
