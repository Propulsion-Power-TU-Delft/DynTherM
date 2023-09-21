within DynTherM.CustomInterfaces;
connector DistributedHeatFluxPort_B "B-type distributed heat flux port connector"
  connector HeatPort = CustomInterfaces.HeatFluxPort_B;
  parameter Integer N(min=1) "Number of ports";
  HeatPort ports[N];
  annotation (Icon(graphics={
                         Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={255,127,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end DistributedHeatFluxPort_B;
