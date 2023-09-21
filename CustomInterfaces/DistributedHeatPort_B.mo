within DynTherM.CustomInterfaces;
connector DistributedHeatPort_B "B-type distributed heat connector"
  connector HeatPort = Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b;
  parameter Integer N(min=1) "Number of ports";
  HeatPort ports[N];
  annotation (Icon(graphics={      Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={191,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end DistributedHeatPort_B;
