within DynTherM.CustomInterfaces;
connector IrradiancePort "1D connector for irradiance"
  Modelica.Units.SI.Irradiance E_tb "Beam component of the clear-sky solar irradiance";
  Modelica.Units.SI.Irradiance E_td "Diffuse component of the clear-sky solar irradiance";
  Modelica.Units.SI.Irradiance E_tr "Ground reflected component of the clear-sky solar irradiance";
  Modelica.Units.SI.Angle theta "Incidence angle";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                  Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={191,0,0},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end IrradiancePort;
