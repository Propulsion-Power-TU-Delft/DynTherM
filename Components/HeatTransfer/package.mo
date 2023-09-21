within DynTherM.Components;
package HeatTransfer "Package collecting all the zero-dimensional components related to heat transfer"

annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
      Rectangle(
        extent={{-90,30},{-30,-30}},
        lineColor={0,0,0},
        fillColor={191,0,0},
        fillPattern=FillPattern.Solid),
      Rectangle(
        extent={{30,30},{90,-30}},
        lineColor={0,0,0},
        fillColor={28,108,200},
        fillPattern=FillPattern.Solid),
      Rectangle(
        extent={{-24,4},{6,-4}},
        lineColor={0,0,0},
        fillColor={0,0,0},
        fillPattern=FillPattern.Solid),
      Polygon(
        points={{6,0},{6,10},{24,0},{6,-10},{6,0}},
        lineColor={0,0,0},
        fillColor={0,0,0},
        fillPattern=FillPattern.Solid),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0)}));
end HeatTransfer;
