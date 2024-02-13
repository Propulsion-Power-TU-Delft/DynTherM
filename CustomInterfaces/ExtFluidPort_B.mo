within DynTherM.CustomInterfaces;
connector ExtFluidPort_B "B-type flow connector using ExternalMedia"
  extends DynTherM.CustomInterfaces.ExtFluidPort;
  annotation (Icon(graphics={Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid), Ellipse(
          extent={{-50,50},{50,-50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end ExtFluidPort_B;
