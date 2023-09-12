within DynTherM.Tests.MassTransfer;
model test_pipe "Simple test of component pipe"
  Modelica.Blocks.Sources.Constant m_fromMix(k=1)
    annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
  Modelica.Blocks.Sources.Constant T_fromMix(k=288.15)
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Modelica.Blocks.Sources.Constant Xw_fromMix(k=0.001)
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
  test_pipe_sys testPipe
    annotation (Placement(transformation(extent={{-20,-16},{20,24}})));
equation
  connect(Xw_fromMix.y, testPipe.Xw_fromMix) annotation (Line(points={{-59,40},
          {-40,40},{-40,8},{-19.2,8}}, color={0,0,127}));
  connect(testPipe.T_fromMix, T_fromMix.y) annotation (Line(points={{-19.2,0},
          {-59,0}},                      color={0,0,127}));
  connect(testPipe.m_fromMix, m_fromMix.y) annotation (Line(points={{-19.2,
          -8},{-40,-8},{-40,-40},{-59,-40}},   color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=1500,
      __Dymola_NumberOfIntervals=100000,
      Tolerance=1e-06));
end test_pipe;
