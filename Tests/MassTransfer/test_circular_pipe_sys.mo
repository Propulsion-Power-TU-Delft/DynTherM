within DynTherM.Tests.MassTransfer;
model test_circular_pipe_sys "Simple test of component pipe"
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1)
    "Refrigerant";

  Modelica.Blocks.Interfaces.RealInput T_fromMix annotation (Placement(
        transformation(extent={{-130,0},{-90,40}}),  iconTransformation(
          extent={{-106,-30},{-86,-10}})));
  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.MassTransfer.SourceMassFlow ECSFlow(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-80,-30})));
  Modelica.Blocks.Interfaces.RealInput m_fromMix annotation (Placement(
        transformation(extent={{-130,-30},{-90,10}}),iconTransformation(
          extent={{-106,-70},{-86,-50}})));
  Components.MassTransfer.PressureSink pressureSink(redeclare package Medium =
        Refrigerant)
    annotation (Placement(transformation(extent={{76,-42},{100,-18}})));
  Components.MassTransfer.CircularPipe pipe(
    redeclare package Medium = Refrigerant,
    allowFlowReversal=false,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    L(displayUnit="mm") = 0.4826,
    D(displayUnit="mm") = 0.005,
    Roughness=0.001e-3)
    annotation (Placement(transformation(extent={{-24,-54},{24,-6}})));

equation
  connect(m_fromMix, ECSFlow.in_massFlow)
    annotation (Line(points={{-110,-10},{-91.2,-10},{-91.2,-20.2}},
                                                           color={0,0,127}));
  connect(T_fromMix, ECSFlow.in_T)
    annotation (Line(points={{-110,20},{-82.8,20},{-82.8,-20.2}},
                                                           color={0,0,127}));
  connect(ECSFlow.outlet, pipe.inlet)
    annotation (Line(points={{-66,-30},{-24,-30}}, color={0,0,0}));
  connect(pipe.outlet, pressureSink.inlet)
    annotation (Line(points={{24,-30},{76,-30}}, color={0,0,0}));
  annotation (Documentation(info="<html>

</html>"), experiment(StopTime=5000, Interval=0.1));
end test_circular_pipe_sys;
