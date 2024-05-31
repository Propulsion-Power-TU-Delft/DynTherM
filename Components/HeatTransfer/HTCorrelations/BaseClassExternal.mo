within DynTherM.Components.HeatTransfer.HTCorrelations;
partial model BaseClassExternal
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  outer DynTherM.Components.Environment environment "Environmental properties";

  input Temperature T_skin "Fuselage outer skin temperature";
  parameter CoefficientOfHeatTransfer ht_start=10
    "Heat transfer coefficient - starting value";

  CoefficientOfHeatTransfer ht(start=ht_start) "Heat transfer coefficient";
  Temperature T_out "Temperature used to compute heat flow rate";

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClassExternal;
