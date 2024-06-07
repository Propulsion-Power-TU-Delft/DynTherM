within DynTherM.Tests.MassTransfer;
model System2
  "Simple test of components SourceMassFlow, Plenum, ValveLin, PressureSink"

  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow cabinHeatFlowRate
    annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={-40,32})));
  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.MassTransfer.Plenum cabin(
    V=186,
    noInitialPressure=false)
    annotation (Placement(transformation(extent={{-54,-44},{-26,-16}})));
  BoundaryConditions.flow_source ECSFlow(
    use_in_massFlow=true,
    use_in_T=true,
    use_di_X=true,
    X_di={0.001,1 - 0.001})
                   annotation (Placement(transformation(
        extent={{16,16},{-16,-16}},
        rotation=180,
        origin={-80,-30})));
  Components.MassTransfer.ValveLin valve(Kv=0.0001) annotation (Placement(
        transformation(
        extent={{-14,-14},{14,14}},
        rotation=0,
        origin={40,-30})));
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
        origin={-60,80})));
  Components.MassTransfer.Fan simpleFan(
    eta_is=0.7,
    eta_m=0.95,
    omega_nom(displayUnit="rpm") = 314.15926535898,
    volFlow_nom=1,
    Head_nom=10000,
    redeclare Components.MassTransfer.FanCharacteristics.Flow.linear flowModel)
    annotation (Placement(transformation(extent={{-16,-46},{16,-14}})));
  BoundaryConditions.mechanical mechanical(
    omega(displayUnit="rpm"),
    use_omega=false,
    use_in_omega=true)
    annotation (Placement(transformation(extent={{-12,-8},{12,8}},
        rotation=-90,
        origin={0,22})));
  Modelica.Blocks.Sources.Ramp valveOpening(
    height=0.5,
    duration=10,
    offset=0.5,
    startTime=500)
    annotation (Placement(transformation(extent={{70,10},{50,30}})));
  Modelica.Blocks.Sources.Ramp fanSpeed(
    height=1000,
    duration=10,
    offset=3000,
    startTime=1000)
    annotation (Placement(transformation(extent={{30,70},{10,90}})));
equation
  connect(cabinHeatFlowRate.port, cabin.thermalPort) annotation (Line(points={{-40,20},
          {-40,-17.4}},                  color={191,0,0}));
  connect(ECSFlow.outlet, cabin.inlet)
    annotation (Line(points={{-64,-30},{-54,-30}}, color={0,0,0}));
  connect(valve.outlet, pressureSink.inlet)
    annotation (Line(points={{54,-30},{66,-30}}, color={0,0,0}));
  connect(T_input.y, ECSFlow.in_T) annotation (Line(points={{-99,60},{-83.2,60},
          {-83.2,-18.8}}, color={0,0,127}));
  connect(Q_input.y, cabinHeatFlowRate.Q_flow)
    annotation (Line(points={{-49,80},{-40,80},{-40,44}}, color={0,0,127}));
  connect(m_input.y, ECSFlow.in_massFlow) annotation (Line(points={{-99,20},{
          -92.8,20},{-92.8,-18.8}}, color={0,0,127}));
  connect(mechanical.mechanical,simpleFan. shaft)
    annotation (Line(points={{0,18},{0,-14}},color={135,135,135}));
  connect(cabin.outlet, simpleFan.inlet)
    annotation (Line(points={{-26,-30},{-16,-30}}, color={0,0,0}));
  connect(simpleFan.outlet, valve.inlet)
    annotation (Line(points={{16,-30},{26,-30}}, color={0,0,0}));
  connect(fanSpeed.y, mechanical.in_omega)
    annotation (Line(points={{9,80},{-8,80},{-8,36.4}}, color={0,0,127}));
  connect(valveOpening.y, valve.opening)
    annotation (Line(points={{49,20},{40,20},{40,-22.44}}, color={0,0,127}));
  annotation (Documentation(info="<html>

</html>"), experiment(StopTime=5000, Interval=0.1),
    Diagram(coordinateSystem(extent={{-120,-100},{100,100}})),
    Icon(coordinateSystem(extent={{-120,-100},{100,100}})));
end System2;
