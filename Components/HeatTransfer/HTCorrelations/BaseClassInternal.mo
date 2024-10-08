within DynTherM.Components.HeatTransfer.HTCorrelations;
partial model BaseClassInternal
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  parameter CoefficientOfHeatTransfer ht_start=10
    "Heat transfer coefficient - starting value";

  CoefficientOfHeatTransfer ht(start=ht_start)
    "Heat transfer coefficient";

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClassInternal;
