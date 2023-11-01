within DynTherM.CustomInterfaces;
connector DistributedHeatPort_A "A-type distributed heat connector"
  connector HeatPort = Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a;
  parameter Integer Nx(min=1) "Number of ports in x-direction";
  parameter Integer Ny(min=1) "Number of ports in y-direction";
  HeatPort ports[Nx,Ny];
  annotation (Icon(graphics={      Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={191,0,0},
          fillColor={191,0,0},
          fillPattern=FillPattern.Solid)}));
end DistributedHeatPort_A;
