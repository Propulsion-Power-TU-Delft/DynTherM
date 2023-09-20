within DynTherM.CustomInterfaces;
connector DistributedHeatPort_A
  connector HeatPort = Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a;
  parameter Integer N(min=1) "Number of ports";
  HeatPort ports[N];
  annotation (Icon(graphics={      Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={191,0,0},
          fillColor={191,0,0},
          fillPattern=FillPattern.Solid)}));
end DistributedHeatPort_A;
