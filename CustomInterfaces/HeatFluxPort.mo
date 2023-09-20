within DynTherM.CustomInterfaces;
connector HeatFluxPort "Heat flux connector"
  Modelica.Units.SI.Temperature T "Port temperature";
  flow Modelica.Units.SI.HeatFlux phi "Heat flux (positive if flowing from outside into the port)";
end HeatFluxPort;
