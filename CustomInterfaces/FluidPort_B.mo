within DynTherM.CustomInterfaces;
connector FluidPort_B "B-type flow connector"
  extends DynTherM.CustomInterfaces.FluidPort;
  annotation (Icon(graphics={Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid), Ellipse(
          extent={{-50,50},{50,-50}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end FluidPort_B;
