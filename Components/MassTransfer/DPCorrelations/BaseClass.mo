within DynTherM.Components.MassTransfer.DPCorrelations;
partial model BaseClass
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  parameter Real f_start=0.03 "Friction factor - starting value";

  Real f(start=f_start) "Friction factor";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClass;
