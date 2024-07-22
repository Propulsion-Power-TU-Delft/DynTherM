within DynTherM.Systems;
package Battery "Models of heat acquisition systems for battery packs"

annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
      Rectangle(
        extent={{-60,78},{60,-100}},
        lineColor={0,0,0},
        fillColor={0,190,95},
        fillPattern=FillPattern.VerticalCylinder),
      Rectangle(
        extent={{-40,90},{40,78}},
        lineColor={0,0,0},
        fillPattern=FillPattern.VerticalCylinder,
        fillColor={215,215,215}),
      Line(
        points={{30,52},{-28,0},{6,-14},{-30,-66},{30,-10},{-2,4},{30,52}},
        color={0,0,0},
        thickness=1)}));
end Battery;
