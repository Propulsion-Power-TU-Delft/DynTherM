within DynTherM.Sensors;
model PressureSensor "Absolute pressure sensor"
  extends Modelica.Icons.RoundSensor;
  Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  CustomInterfaces.FluidPort_A port
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
equation
  port.m_flow = 0;
  port.h_outflow = 0;
  port.Xi_outflow = zeros(2);

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
