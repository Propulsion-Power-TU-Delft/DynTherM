within DynTherM;
package Systems

  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
        Ellipse(extent={{-34,34},{34,-34}}, lineColor={0,0,0}),
        Ellipse(extent={{-80,60},{-60,40}}, lineColor={0,0,0}),
        Ellipse(extent={{60,60},{80,40}}, lineColor={0,0,0}),
        Ellipse(
          extent={{-10,90},{10,70}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-60,-60},{-40,-80}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{40,-60},{60,-80}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(points={{0,34},{0,70}}, color={0,0,0}),
        Line(points={{-28,20},{-62,44}}, color={0,0,0}),
        Line(points={{28,20},{62,44}}, color={0,0,0}),
        Line(points={{-20,-28},{-44,-62}}, color={0,0,0}),
        Line(points={{20,-28},{44,-62}}, color={0,0,0})}));
end Systems;
