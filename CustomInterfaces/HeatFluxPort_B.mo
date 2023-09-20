within DynTherM.CustomInterfaces;
connector HeatFluxPort_B "B-type heat flux connector"
  extends HeatFluxPort;
  annotation (Icon(graphics={
                         Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={255,127,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end HeatFluxPort_B;
