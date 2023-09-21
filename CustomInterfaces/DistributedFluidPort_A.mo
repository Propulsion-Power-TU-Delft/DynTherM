within DynTherM.CustomInterfaces;
connector DistributedFluidPort_A "A-type distributed flow connector"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  connector FluidPort = DynTherM.CustomInterfaces.FluidPort_A (
    redeclare package Medium=Medium);
  parameter Integer N(min=1) "Number of ports";
  FluidPort ports[N];
  annotation (Icon(graphics={Ellipse(
          extent={{-100,-20},{100,20}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}));
end DistributedFluidPort_A;
