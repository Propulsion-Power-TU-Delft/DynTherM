within DynTherM.Sensors;
model TemperatureDifferentialSensor "Temperature differential sensor"
  extends Modelica.Icons.RoundSensor;
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
  Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{10,96},
            {30,116}}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,110})));
  CustomInterfaces.ZeroDimensional.FluidPort_A inlet(redeclare package Medium =
        Medium, m_flow(min=if allowFlowReversal then -Modelica.Constants.inf
           else 0))
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  CustomInterfaces.ZeroDimensional.FluidPort_B outlet(redeclare package Medium =
        Medium, m_flow(max=if allowFlowReversal then +Modelica.Constants.inf
           else 0)) annotation (Placement(transformation(extent={{90,-10},{110,
            10}}), iconTransformation(extent={{90,-10},{110,10}})));

equation
  // Zero flow equations for connectors
  inlet.m_flow = 0;
  outlet.m_flow = 0;

  // No contribution of specific quantities
  inlet.h_outflow = Medium.h_default;
  outlet.h_outflow = Medium.h_default;
  inlet.Xi_outflow = Medium.X_default[1:Medium.nX];
  outlet.Xi_outflow = Medium.X_default[1:Medium.nX];

  // Relative pressure
  y = Medium.temperature(Medium.setState_phX(inlet.P, inStream(inlet.h_outflow), inStream(inlet.Xi_outflow))) -
      Medium.temperature(Medium.setState_phX(outlet.P, inStream(outlet.h_outflow), inStream(outlet.Xi_outflow)));

  annotation (
    Documentation(info="<html>
<p>The PressureSensor measures the absolute pressure.</p>
<p>Thermodynamic equations are defined by Partials.AbsoluteSensor.</p>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={           Line(points={{-100,0},{-70,0}}, color={0,0,0}),
                           Line(points={{0,70},{0,100}},   color={0,0,0}),
                           Line(points={{70,0},{100,0}},   color={0,0,0}),
                Text(
          extent={{-30,-8},{28,-66}},
          lineColor={0,0,0},
          textString="dT")}));
end TemperatureDifferentialSensor;
