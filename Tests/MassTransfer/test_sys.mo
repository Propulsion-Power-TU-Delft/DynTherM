within DynTherM.Tests.MassTransfer;
model test_sys
  "Simple test of components SourceMassFlow, Plenum, ValveLin, PressureSink"

  Modelica.Blocks.Interfaces.RealInput T_fromMix annotation (Placement(
        transformation(extent={{-130,0},{-90,40}}),  iconTransformation(
          extent={{-106,-30},{-86,-10}})));
  Modelica.Blocks.Interfaces.RealInput Q_cabin annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-10,100}),
                         iconTransformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={-20,92})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow cabinHeatFlowRate
    annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={-10,60})));
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
    use_in_Xw=true)
                   annotation (Placement(transformation(
        extent={{16,16},{-16,-16}},
        rotation=180,
        origin={-64,-30})));
  Modelica.Blocks.Interfaces.RealInput Xw_fromMix annotation (Placement(
        transformation(extent={{-130,30},{-90,70}}),  iconTransformation(
          extent={{-106,10},{-86,30}})));
  Components.MassTransfer.ValveLin valve(Kv=0.0001) annotation (Placement(
        transformation(
        extent={{-14,-14},{14,14}},
        rotation=0,
        origin={34,-30})));
  Modelica.Blocks.Interfaces.RealInput m_fromMix annotation (Placement(
        transformation(extent={{-130,-30},{-90,10}}),iconTransformation(
          extent={{-106,-70},{-86,-50}})));
  BoundaryConditions.pressure_sink pressureSink
    annotation (Placement(transformation(extent={{66,-44},{94,-16}})));
  Modelica.Blocks.Interfaces.RealInput valveOpening annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={34,100}), iconTransformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={20,92})));
equation
  connect(Q_cabin, cabinHeatFlowRate.Q_flow)
    annotation (Line(points={{-10,100},{-10,72}},
                                              color={0,0,127}));
  connect(cabinHeatFlowRate.port, cabin.thermalPort) annotation (Line(points={{-10,48},
          {-10,-17.4}},                  color={191,0,0}));
  connect(ECSFlow.outlet, cabin.inlet)
    annotation (Line(points={{-48,-30},{-24,-30}}, color={0,0,0}));
  connect(cabin.outlet, valve.inlet)
    annotation (Line(points={{4,-30},{20,-30}}, color={0,0,0}));
  connect(valve.outlet, pressureSink.inlet)
    annotation (Line(points={{48,-30},{66,-30}}, color={0,0,0}));
  connect(valveOpening, valve.opening)
    annotation (Line(points={{34,100},{34,-22.44}}, color={0,0,127}));
  connect(m_fromMix, ECSFlow.in_massFlow) annotation (Line(points={{-110,-10},{-76.8,
          -10},{-76.8,-18.8}}, color={0,0,127}));
  connect(T_fromMix, ECSFlow.in_T) annotation (Line(points={{-110,20},{-67.2,20},
          {-67.2,-18.8}},
                       color={0,0,127}));
  connect(Xw_fromMix, ECSFlow.in_Xw) annotation (Line(points={{-110,50},{-57.6,50},
          {-57.6,-18.8}}, color={0,0,127}));
  annotation (Documentation(info="<html>

</html>"), experiment(StopTime=5000, Interval=0.1));
end test_sys;
