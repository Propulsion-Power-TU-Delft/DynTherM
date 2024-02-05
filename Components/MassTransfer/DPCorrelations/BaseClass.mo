within DynTherM.Components.MassTransfer.DPCorrelations;
partial model BaseClass
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  parameter Integer Nx(min=1) "Number of control volumes in x-direction";
  parameter Integer Ny(min=1) "Number of control volumes in y-direction";
  parameter Real f_start[Nx,Ny]=0.03*ones(Nx,Ny) "Friction factor - starting value";

  Real f[Nx,Ny](start=f_start) "Friction factor";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClass;
