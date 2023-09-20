within DynTherM.Components;
package TwoDimensional
  "Package collecting the components modeling coupled heat and mass transfer and featuring a two dimensional spatial discretization"

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
          radius=25.0),      Text(
          extent={{-94,98},{98,-94}},
          lineColor={0,0,0},
          textString="2D")}));
end TwoDimensional;
