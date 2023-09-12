within DynTherM.CustomInterfaces;
connector Shaft_B "B-type mechanical connector"
  extends MechanicalPort;
  annotation (Icon(graphics={                                                                                     Polygon(
          points={{-100,40},{-40,100},{40,100},{100,40},{100,-40},{40,
              -100},{-40,-100},{-100,-40},{-100,40}},
          lineColor={135,135,135},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1)}));
end Shaft_B;
