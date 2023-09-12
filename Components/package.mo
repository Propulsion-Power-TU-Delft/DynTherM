within DynTherM;
package Components

  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
        Rectangle(extent={{-80,80},{-12,12}}, lineColor={0,0,0}),
        Rectangle(
          extent={{12,80},{80,12}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(extent={{12,-12},{80,-80}}, lineColor={0,0,0}),
        Rectangle(extent={{-80,-12},{-12,-80}}, lineColor={0,0,0}),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0)}));
end Components;
