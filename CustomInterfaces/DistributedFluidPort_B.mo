within DynTherM.CustomInterfaces;
connector DistributedFluidPort_B "B-type distributed flow connector"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  connector FluidPort = DynTherM.CustomInterfaces.FluidPort_B (
    redeclare package Medium=Medium);
  parameter Integer N(min=1) "Number of ports";
  FluidPort ports[N];
  annotation (Icon(graphics={Ellipse(
          extent={{-100,-20},{100,20}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                             Ellipse(
          extent={{-86,-12},{86,12}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end DistributedFluidPort_B;
