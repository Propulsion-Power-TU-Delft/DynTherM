within DynTherM.CustomInterfaces;
connector HeatFluxPort_A "A-type heat flux connector"
  extends HeatFluxPort;
  annotation (Icon(graphics={
                         Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={255,127,0},
          fillColor={255,127,0},
          fillPattern=FillPattern.Solid)}));
end HeatFluxPort_A;
