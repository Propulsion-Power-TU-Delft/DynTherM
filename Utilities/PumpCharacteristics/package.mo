within DynTherM.Utilities;
package PumpCharacteristics "Package collecting different laws describing the characteristic curves of pumps"

  annotation (Icon(graphics={
        Line(
          points={{-80,-80},{-80,100}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{-80,-80},{100,-80}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{-80,100},{-94,80}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{-80,100},{-66,80}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{100,-80},{80,-66}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{100,-80},{80,-94}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{20,-80},{20,4},{-80,4}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{-80,60},{0,28},{36,-16},{68,-80}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Ellipse(
          extent={{16,8},{24,0}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid)}));
end PumpCharacteristics;
