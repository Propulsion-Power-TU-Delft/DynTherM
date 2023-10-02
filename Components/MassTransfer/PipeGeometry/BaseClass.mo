within DynTherM.Components.MassTransfer.PipeGeometry;
partial model BaseClass
  parameter Modelica.Units.SI.Length L "Length";
  Modelica.Units.SI.Area A_cs "Cross-sectional area";
  Modelica.Units.SI.Area A_ht "Heat transfer area";
  Modelica.Units.SI.Length P_cs "Cross-sectional perimeter";
  Modelica.Units.SI.Length Dh "Hydraulic diameter";

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClass;
