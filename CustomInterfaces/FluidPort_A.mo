within DynTherM.CustomInterfaces;
connector FluidPort_A "A-type connector for moist air flows"
  extends DynTherM.CustomInterfaces.FluidPort;
  annotation (Icon(graphics={Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}));
end FluidPort_A;
