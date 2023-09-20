within DynTherM.CustomInterfaces;
connector DistributedHeatFluxPort_A
  connector HeatPort = CustomInterfaces.HeatFluxPort_A;
  parameter Integer N(min=1) "Number of ports";
  HeatPort ports[N];
  annotation (Icon(graphics={
                         Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={255,127,0},
          fillColor={255,127,0},
          fillPattern=FillPattern.Solid)}));
end DistributedHeatFluxPort_A;
