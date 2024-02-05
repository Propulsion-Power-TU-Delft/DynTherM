within DynTherM.Components.HeatTransfer.HTCorrelations;
partial model BaseClassExternal
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  outer DynTherM.Components.Environment environment "Environmental properties";

  parameter Integer Nx(min=1) "Number of control volumes in x-direction";
  parameter Integer Ny(min=1) "Number of control volumes in y-direction";
  input Modelica.Units.SI.Temperature T_skin[Nx,Ny]
    "Fuselage outer skin temperature";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_start[Nx,Ny]=10*ones(Nx,Ny)
    "Heat transfer coefficient - starting value";

  Modelica.Units.SI.CoefficientOfHeatTransfer ht[Nx,Ny](start=ht_start)
    "Heat transfer coefficient";
  Modelica.Units.SI.Temperature T_out[Nx,Ny]
    "Temperature used to compute heat flow rate";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClassExternal;
