within DynTherM.CustomInterfaces;
connector DistributedHeatFluxPort_A "A-type distributed heat flux connector"
  connector HeatPort = CustomInterfaces.HeatFluxPort_A;
  parameter Integer Nx(min=1) "Number of ports in x-direction";
  parameter Integer Ny(min=1) "Number of ports in y-direction";
  HeatPort ports[Nx,Ny];
  annotation (Icon(graphics={
                         Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={255,127,0},
          fillColor={255,127,0},
          fillPattern=FillPattern.Solid)}));
end DistributedHeatFluxPort_A;
