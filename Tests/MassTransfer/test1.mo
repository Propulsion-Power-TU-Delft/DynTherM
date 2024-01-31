within DynTherM.Tests.MassTransfer;
model test1
  "Simple test of components SourceMassFlow, Plenum, ValveLin, PressureSink"

  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow cabinHeatFlowRate
    annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={-10,32})));
  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.MassTransfer.Plenum cabin(
    V=186,
    noInitialPressure=false)
    annotation (Placement(transformation(extent={{-24,-44},{4,-16}})));
  BoundaryConditions.flow_source ECSFlow(
    use_in_massFlow=true,
    use_in_T=true,
    use_di_X=true,
    X_di={0.001,1 - 0.001})
                   annotation (Placement(transformation(
        extent={{16,16},{-16,-16}},
        rotation=180,
        origin={-64,-30})));
  Components.MassTransfer.ValveLin valve(Kv=0.0001) annotation (Placement(
        transformation(
        extent={{-14,-14},{14,14}},
        rotation=0,
        origin={34,-30})));
  BoundaryConditions.pressure_sink pressureSink
    annotation (Placement(transformation(extent={{66,-44},{94,-16}})));
  Modelica.Blocks.Sources.Constant m_input(k=1)
    annotation (Placement(transformation(extent={{-120,10},{-100,30}})));
  Modelica.Blocks.Sources.Constant T_input(k=288.15)
    annotation (Placement(transformation(extent={{-120,50},{-100,70}})));
  Modelica.Blocks.Sources.Constant Q_input(k=10000) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,80})));
  Modelica.Blocks.Sources.Constant dummy_valve(k=1) annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={60,40})));
equation
  connect(cabinHeatFlowRate.port, cabin.thermalPort) annotation (Line(points={{-10,20},
          {-10,1.3},{-10,1.3},{-10,-17.4}},
                                         color={191,0,0}));
  connect(ECSFlow.outlet, cabin.inlet)
    annotation (Line(points={{-48,-30},{-24,-30}}, color={0,0,0}));
  connect(cabin.outlet, valve.inlet)
    annotation (Line(points={{4,-30},{20,-30}}, color={0,0,0}));
  connect(valve.outlet, pressureSink.inlet)
    annotation (Line(points={{48,-30},{66,-30}}, color={0,0,0}));
  connect(m_input.y, ECSFlow.in_massFlow) annotation (Line(points={{-99,20},{
          -76,20},{-76,-18.8},{-76.8,-18.8}}, color={0,0,127}));
  connect(T_input.y, ECSFlow.in_T) annotation (Line(points={{-99,60},{-67.2,60},
          {-67.2,-18.8}}, color={0,0,127}));
  connect(dummy_valve.y, valve.opening)
    annotation (Line(points={{49,40},{34,40},{34,-22.44}}, color={0,0,127}));
  connect(Q_input.y, cabinHeatFlowRate.Q_flow)
    annotation (Line(points={{-19,80},{-10,80},{-10,44}}, color={0,0,127}));
  annotation (Documentation(info="<html>

</html>"), experiment(StopTime=5000, Interval=0.1),
    Diagram(coordinateSystem(extent={{-120,-100},{100,100}})),
    Icon(coordinateSystem(extent={{-120,-100},{100,100}})));
end test1;
