within ThermalManagement.Tests.MassTransfer;
model test_fan "Simple test of component simpleFan"
  Modelica.Blocks.Sources.Constant m_fromMix(k=1)
    annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
  Modelica.Blocks.Sources.Constant Q_cabin(k=10000) annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-30,80})));
  Modelica.Blocks.Sources.Constant T_fromMix(k=288.15)
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Modelica.Blocks.Sources.Constant Xw_fromMix(k=0.001)
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
  test_fan_sys moistAir_testFan
    annotation (Placement(transformation(extent={{0,-16},{40,24}})));
  Modelica.Blocks.Sources.Ramp valveOpening(
    height=0.5,
    duration=10,
    offset=0.5,
    startTime=500)
    annotation (Placement(transformation(extent={{70,70},{50,90}})));
  Modelica.Blocks.Sources.Ramp fanSpeed(
    height=1000,
    duration=10,
    offset=3000,
    startTime=1000)
    annotation (Placement(transformation(extent={{80,30},{60,50}})));
equation
  connect(Xw_fromMix.y, moistAir_testFan.Xw_fromMix) annotation (Line(points={{-59,40},
          {-20,40},{-20,8},{0.8,8}},             color={0,0,127}));
  connect(T_fromMix.y, moistAir_testFan.T_fromMix) annotation (Line(points={{-59,0},
          {0.8,0}},                                color={0,0,127}));
  connect(m_fromMix.y, moistAir_testFan.m_fromMix) annotation (Line(points={{-59,-40},
          {-20,-40},{-20,-8},{0.8,-8}},              color={0,0,127}));
  connect(Q_cabin.y, moistAir_testFan.Q_cabin) annotation (Line(points={{-19,80},
          {13.6,80},{13.6,22.4}},               color={0,0,127}));
  connect(valveOpening.y, moistAir_testFan.valveOpening)
    annotation (Line(points={{49,80},{20,80},{20,22.4}}, color={0,0,127}));
  connect(fanSpeed.y, moistAir_testFan.fanSpeed)
    annotation (Line(points={{59,40},{28,40},{28,22.4}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=1500,
      __Dymola_NumberOfIntervals=100000,
      Tolerance=1e-06));
end test_fan;
