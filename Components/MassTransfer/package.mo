within DynTherM.Components;
package MassTransfer "Package collecting all the zero-dimensional components related to mass transfer"

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
          radius=25),
      Line(points={{-100,-60},{100,-60}}, color={128,128,128}),
      Line(points={{-100,60},{100,60}}, color={128,128,128}),
      Line(
        points={{0,60},{-34,18},{-34,-18},{0,-60}},
        color={128,128,128},
        smooth=Smooth.Bezier),
      Ellipse(
        extent={{-50,-16},{-34,-32}},
        lineColor={0,0,0},
        fillColor={0,0,0},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-14,-14},{2,-30}},
        lineColor={0,0,0},
        fillColor={0,0,0},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-14,28},{2,12}},
        lineColor={0,0,0},
        fillColor={0,0,0},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-30,58},{-14,42}},
        lineColor={0,0,0},
        fillColor={0,0,0},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-30,-42},{-14,-58}},
        lineColor={0,0,0},
        fillColor={0,0,0},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-50,30},{-34,14}},
        lineColor={0,0,0},
        fillColor={0,0,0},
        fillPattern=FillPattern.Solid),
      Polygon(
        points={{-20,-2},{-20,20},{20,0},{-20,-20},{-20,-2}},
        lineColor={0,128,255},
        smooth=Smooth.None,
        fillColor={0,128,255},
        fillPattern=FillPattern.Solid,
        origin={43,0},
        rotation=0),
      Rectangle(
        extent={{-50.5,7},{50.5,-7}},
        lineColor={0,128,255},
        fillColor={0,128,255},
        fillPattern=FillPattern.Solid,
        origin={-27.5,-1},
        rotation=0)}));
end MassTransfer;
