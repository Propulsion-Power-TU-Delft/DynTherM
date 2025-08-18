within DynTherM.Sensors;
model PressureSensor "Absolute pressure sensor"
  extends Modelica.Icons.RoundSensor;
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  CustomInterfaces.ZeroDimensional.FluidPort_A port(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
equation
  port.m_flow = 0;
  port.h_outflow = Medium.h_default;
  port.Xi_outflow = Medium.X_default[1:Medium.nX];

  // sensor output
  y = port.P;
  annotation (
    Documentation(info="<html>
<p>The PressureSensor measures the absolute pressure.</p>
<p>Thermodynamic equations are defined by Partials.AbsoluteSensor.</p>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={Text(
          extent={{-30,-20},{30,-60}},
          textString="P",
          lineColor={0,0,0}),
                           Line(points={{-100,0},{-70,0}}, color={0,0,0}),
                           Line(points={{70,0},{100,0}},   color={0,0,0})}));
end PressureSensor;
