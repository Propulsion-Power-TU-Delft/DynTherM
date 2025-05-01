within DynTherM.Sensors;
model PressureSensorExt
  "Absolute pressure sensor for ExternalMedia fluid"
  extends Modelica.Icons.RoundSensor;
  replaceable package Medium = Media.ExtMedia.CoolProp.Hydrogen constrainedby
    ExternalMedia.Media.BaseClasses.ExternalTwoPhaseMedium "Medium model" annotation(choicesAllMatching = true);
  Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  CustomInterfaces.ZeroDimensional.ExtFluidPort_A port(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
equation
  port.m_flow = 0;
  port.h_outflow = 0;
  port.Xi_outflow = zeros(Medium.nX);
  port.C_outflow = zeros(Medium.nC);

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
end PressureSensorExt;
