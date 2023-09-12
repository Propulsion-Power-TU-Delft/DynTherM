within ThermalManagement.Components.HeatTransfer.HTCorrelations;
partial model BaseClassInternal
  package Medium = Modelica.Media.Air.MoistAir;
  outer ThermalManagement.Components.Environment environment
    "Environmental properties";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_start=10
    "Heat transfer coefficient - starting value";
  Modelica.Units.SI.CoefficientOfHeatTransfer ht(start=ht_start)
    "Heat transfer coefficient";

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClassInternal;
