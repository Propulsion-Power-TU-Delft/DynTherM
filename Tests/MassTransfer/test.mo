within DynTherM.Tests.MassTransfer;
model test
  "Simple test of components SourceMassFlow, Plenum, ValveLin, PressureSink"
  Modelica.Blocks.Sources.Constant m_fromMix(k=1)
    annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
  test_sys test1
    annotation (Placement(transformation(extent={{0,-16},{40,24}})));
  Modelica.Blocks.Sources.Constant Q_cabin(k=10000) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,80})));
  Modelica.Blocks.Sources.Constant fakeValve(k=1)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=0,
        origin={70,80})));
  Modelica.Blocks.Sources.Constant T_fromMix(k=288.15)
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Modelica.Blocks.Sources.Constant Xw_fromMix(k=0.001)
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
equation
  connect(Xw_fromMix.y, test1.Xw_fromMix) annotation (Line(points={{-59,40},
          {-20,40},{-20,8},{0.8,8}},
                                color={0,0,127}));
  connect(T_fromMix.y, test1.T_fromMix) annotation (Line(points={{-59,0},{
          0.8,0}},                 color={0,0,127}));
  connect(m_fromMix.y, test1.m_fromMix) annotation (Line(points={{-59,-40},
          {-20,-40},{-20,-8},{0.8,-8}},
                                     color={0,0,127}));
  connect(Q_cabin.y, test1.Q_cabin) annotation (Line(points={{-19,80},{16,
          80},{16,22.4}},    color={0,0,127}));
  connect(fakeValve.y, test1.valveOpening) annotation (Line(points={{59,80},
          {24,80},{24,22.4}},     color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=1000,
      __Dymola_NumberOfIntervals=100000,
      Tolerance=1e-06));
end test;
