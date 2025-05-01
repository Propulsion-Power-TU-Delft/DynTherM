within DynTherM.Sensors;
model MassflowSensorExt
  "Mass flow rate sensor for ExternalMedia fluid"
  extends Modelica.Icons.RoundSensor;
  replaceable package Medium = Media.ExtMedia.CoolProp.Hydrogen constrainedby
    ExternalMedia.Media.BaseClasses.ExternalTwoPhaseMedium "Medium model" annotation(choicesAllMatching = true);
  outer DynTherM.Components.Environment environment "Environmental properties";
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{10,96},
            {30,116}}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,110})));
  CustomInterfaces.ZeroDimensional.ExtFluidPort_A inlet(redeclare package
      Medium = Medium, m_flow(min=if allowFlowReversal then -Modelica.Constants.inf
           else 0))
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  CustomInterfaces.ZeroDimensional.ExtFluidPort_B outlet(redeclare package
      Medium = Medium, m_flow(max=if allowFlowReversal then +Modelica.Constants.inf
           else 0)) annotation (Placement(transformation(extent={{90,-10},{110,
            10}}), iconTransformation(extent={{90,-10},{110,10}})));
equation
  inlet.m_flow + outlet.m_flow = 0 "Mass balance";
  inlet.P = outlet.P "Momentum balance";

  // Energy balance
  inlet.h_outflow = inStream(outlet.h_outflow);
  outlet.h_outflow = inStream(inlet.h_outflow);

  // Independent composition mass balances
  inlet.Xi_outflow = inStream(outlet.Xi_outflow);
  outlet.Xi_outflow = inStream(inlet.Xi_outflow);

  // Properties c_i/m balance
  inlet.C_outflow = inStream(outlet.C_outflow);
  outlet.C_outflow = inStream(inlet.C_outflow);

  y = inlet.m_flow "Sensor output";
  annotation (
    Documentation(info="<html>
<p>The PressureSensor measures the absolute pressure.</p>
<p>Thermodynamic equations are defined by Partials.AbsoluteSensor.</p>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={Text(
          extent={{-50,18},{50,-82}},
          lineColor={0,0,0},
          textString="m flow"),
                           Line(points={{-100,0},{-70,0}}, color={0,0,0}),
                           Line(points={{0,70},{0,100}},   color={0,0,0}),
                           Line(points={{70,0},{100,0}},   color={0,0,0})}));
end MassflowSensorExt;
