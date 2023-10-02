within DynTherM;
package Examples "Collection of exemplary models built with DynTherM"



  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
        Ellipse(
          extent={{-56,80},{56,-32}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.None),
        Line(points={{-70,-48},{74,-48}}, color={0,0,0}),
        Line(points={{-70,-66},{74,-66}}, color={0,0,0}),
        Line(points={{-70,-84},{48,-84}}, color={0,0,0}),
        Line(points={{-32,50},{0,18}}, color={0,0,0}),
        Line(points={{0,18},{66,84}}, color={0,0,0}),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0)}));
end Examples;
