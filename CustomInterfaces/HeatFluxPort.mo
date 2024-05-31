within DynTherM.CustomInterfaces;
connector HeatFluxPort "Heat flux connector"
  Temperature T "Port temperature";
  flow HeatFlux phi "Heat flux (positive if flowing from outside into the port)";
end HeatFluxPort;
