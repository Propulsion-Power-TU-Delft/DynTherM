within DynTherM.Components.OneDimensional;
model HeatCapacity1D
  "Heat capacity model implementing 1D spatial discretization"
  outer DynTherM.Components.Environment environment
    "Environmental properties";

  parameter DynTherM.Choices.InitOpt initOpt=environment.initOpt
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialTemperature=false
    "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Temperature T_start=288.15 "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.HeatCapacity C "Heat capacity";
  parameter Integer N_cv=1 "Number of control volumes";

  model CV = DynTherM.Components.HeatTransfer.HeatCapacity "Control volume";

  CV cv[N_cv](
    each C=C/N_cv,
    each T_start=T_start,
    each initOpt=initOpt,
    each noInitialTemperature=noInitialTemperature);

  CustomInterfaces.DistributedHeatPort_A thermal(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(extent={{-40,-132},{40,-52}}),
        iconTransformation(extent={{-40,-132},{40,-52}})));

equation
  for i in 1:N_cv loop
    connect(thermal.ports[i,1], cv[i].port);
  end for;

  annotation (
      Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
              100,100}}), graphics={
          Text(
            extent={{-150,110},{150,70}},
            textString="%name",
            lineColor={0,0,255}),
          Polygon(
            points={{0,67},{-20,63},{-40,57},{-52,43},{-58,35},{-68,25},{-72,
                13},{-76,-1},{-78,-15},{-76,-31},{-76,-43},{-76,-53},{-70,-65},
                {-64,-73},{-48,-77},{-30,-83},{-18,-83},{-2,-85},{8,-89},{22,
                -89},{32,-87},{42,-81},{54,-75},{56,-73},{66,-61},{68,-53},{
                70,-51},{72,-35},{76,-21},{78,-13},{78,3},{74,15},{66,25},{54,
                33},{44,41},{36,57},{26,65},{0,67}},
            lineColor={160,160,164},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-58,35},{-68,25},{-72,13},{-76,-1},{-78,-15},{-76,-31},{
                -76,-43},{-76,-53},{-70,-65},{-64,-73},{-48,-77},{-30,-83},{-18,
                -83},{-2,-85},{8,-89},{22,-89},{32,-87},{42,-81},{54,-75},{42,
                -77},{40,-77},{30,-79},{20,-81},{18,-81},{10,-81},{2,-77},{-12,
                -73},{-22,-73},{-30,-71},{-40,-65},{-50,-55},{-56,-43},{-58,-35},
                {-58,-25},{-60,-13},{-60,-5},{-60,7},{-58,17},{-56,19},{-52,
                27},{-48,35},{-44,45},{-40,57},{-58,35}},
            fillColor={160,160,164},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-69,7},{71,-24}},
            textString="%C")}),
      Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
              {100,100}}), graphics={
          Polygon(
            points={{0,67},{-20,63},{-40,57},{-52,43},{-58,35},{-68,25},{-72,
                13},{-76,-1},{-78,-15},{-76,-31},{-76,-43},{-76,-53},{-70,-65},
                {-64,-73},{-48,-77},{-30,-83},{-18,-83},{-2,-85},{8,-89},{22,
                -89},{32,-87},{42,-81},{54,-75},{56,-73},{66,-61},{68,-53},{
                70,-51},{72,-35},{76,-21},{78,-13},{78,3},{74,15},{66,25},{54,
                33},{44,41},{36,57},{26,65},{0,67}},
            lineColor={160,160,164},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-58,35},{-68,25},{-72,13},{-76,-1},{-78,-15},{-76,-31},{
                -76,-43},{-76,-53},{-70,-65},{-64,-73},{-48,-77},{-30,-83},{-18,
                -83},{-2,-85},{8,-89},{22,-89},{32,-87},{42,-81},{54,-75},{42,
                -77},{40,-77},{30,-79},{20,-81},{18,-81},{10,-81},{2,-77},{-12,
                -73},{-22,-73},{-30,-71},{-40,-65},{-50,-55},{-56,-43},{-58,-35},
                {-58,-25},{-60,-13},{-60,-5},{-60,7},{-58,17},{-56,19},{-52,
                27},{-48,35},{-44,45},{-40,57},{-58,35}},
            fillColor={160,160,164},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-6,-1},{6,-12}},
            lineColor={255,0,0},
            fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{11,13},{50,-25}},
            textString="T"),
          Line(points={{0,-12},{0,-96}}, color={255,0,0})}),
      Documentation(info="<html>
</html>"));
end HeatCapacity1D;
