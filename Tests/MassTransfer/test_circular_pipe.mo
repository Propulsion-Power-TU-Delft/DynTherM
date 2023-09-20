within DynTherM.Tests.MassTransfer;
model test_circular_pipe "Simple test of component pipe"
  Modelica.Blocks.Sources.Constant m_fromMix(k=0.3)
    annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
  Modelica.Blocks.Sources.Constant T_fromMix(k=273.15 + 50)
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  test_circular_pipe_sys testPipe
    annotation (Placement(transformation(extent={{-20,-16},{20,24}})));
equation
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
end test_circular_pipe;
