within DynTherM.CustomInterfaces;
connector IrradiancePort "Irradiance connector"
  Irradiance E_tb "Beam component of the clear-sky solar irradiance";
  Irradiance E_td "Diffuse component of the clear-sky solar irradiance";
  Irradiance E_tr "Ground reflected component of the clear-sky solar irradiance";
  Angle theta "Incidence angle";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                  Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={191,0,0},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid), Ellipse(
          extent={{-72,72},{72,-72}},
          lineColor={191,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end IrradiancePort;
