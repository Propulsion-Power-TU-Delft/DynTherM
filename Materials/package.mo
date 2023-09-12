within ThermalManagement;
package Materials "Materials Library for Aerospace Applications"

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
        Line(
          points={{-86,-92},{-64,-46},{-58,0},{-64,48},{-86,90}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Line(
          points={{-86,90},{-40,74},{0,68},{40,74},{86,90}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Line(
          points={{86,-92},{64,-46},{58,0},{64,48},{86,90}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Line(
          points={{-86,-92},{-40,-76},{0,-70},{40,-76},{86,-92}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Line(
          points={{0,-72},{0,-40}},
          color={238,46,47},
          smooth=Smooth.Bezier),
        Line(points={{0,-40},{-8,-50}}, color={238,46,47}),
        Line(points={{0,-40},{8,-50}}, color={238,46,47}),
        Line(
          points={{0,70},{0,38}},
          color={238,46,47},
          smooth=Smooth.Bezier),
        Line(points={{0,38},{-8,48}}, color={238,46,47}),
        Line(points={{0,38},{8,48}}, color={238,46,47}),
        Line(
          points={{0,-16},{-2.54946e-15,24}},
          color={238,46,47},
          smooth=Smooth.Bezier,
          origin={-44,0},
          rotation=-90),
        Line(points={{-20,0},{-32,10}}, color={238,46,47}),
        Line(points={{-20,0},{-32,-8}}, color={238,46,47}),
        Line(
          points={{4.02646e-15,-24},{0,16}},
          color={238,46,47},
          smooth=Smooth.Bezier,
          origin={44,0},
          rotation=-90),
        Line(points={{32,-10},{20,0}}, color={238,46,47}),
        Line(points={{32,8},{20,0}}, color={238,46,47}),
        Line(points={{-86,90},{-50,56}}, color={238,46,47}),
        Line(points={{50,-58},{86,-92}}, color={238,46,47}),
        Line(points={{-50,-58},{-86,-92}}, color={238,46,47}),
        Line(points={{86,90},{50,56}}, color={238,46,47}),
        Line(points={{84,76},{86,90}}, color={238,46,47}),
        Line(points={{86,90},{72,90}}, color={238,46,47}),
        Line(points={{-72,90},{-86,90}}, color={238,46,47}),
        Line(points={{-72,-92},{-86,-92}}, color={238,46,47}),
        Line(points={{86,-92},{72,-92}}, color={238,46,47}),
        Line(points={{-86,-92},{-84,-78}}, color={238,46,47}),
        Line(points={{-84,76},{-86,90}}, color={238,46,47}),
        Line(points={{86,-92},{84,-78}}, color={238,46,47})}));
end Materials;
