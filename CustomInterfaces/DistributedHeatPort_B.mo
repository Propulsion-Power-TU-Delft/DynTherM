within DynTherM.CustomInterfaces;
connector DistributedHeatPort_B "B-type distributed heat connector"
  connector HeatPort = Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b;
  parameter Integer Nx(min=1) "Number of ports in x-direction";
  parameter Integer Ny(min=1) "Number of ports in y-direction";
  HeatPort ports[Nx,Ny];
  annotation (Icon(graphics={      Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={191,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end DistributedHeatPort_B;
