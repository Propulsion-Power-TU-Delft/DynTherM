within DynTherM.CustomInterfaces;
connector DistributedHeatPort "Distributed heat port (1D)"
  parameter Integer N "Number of ports";
  Modelica.Units.SI.Temperature T[N] "Ports' temperature";
  flow Modelica.Units.SI.HeatFlowRate Q_flow[N] "Heat flow rate (positive if flowing from outside into the ports)";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end DistributedHeatPort;
