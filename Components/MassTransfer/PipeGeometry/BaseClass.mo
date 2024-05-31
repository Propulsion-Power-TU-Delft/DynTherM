within DynTherM.Components.MassTransfer.PipeGeometry;
partial model BaseClass
  parameter Length L "Length";

  Area A_cs "Cross-sectional area";
  Area A_ht "Heat transfer area";
  Length P_cs "Cross-sectional perimeter";
  Length Dh "Hydraulic diameter";

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClass;
