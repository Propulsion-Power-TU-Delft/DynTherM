within ThermalManagement.Tests.MassTransfer;
model test_fan_sys "Simple test of component simpleFan"

  Modelica.Blocks.Interfaces.RealInput T_fromMix annotation (Placement(
        transformation(extent={{-130,0},{-90,40}}),  iconTransformation(
          extent={{-106,-30},{-86,-10}})));
  Modelica.Blocks.Interfaces.RealInput Q_cabin annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-40,100}),
                         iconTransformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={-32,92})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow cabinHeatFlowRate
    annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={-40,60})));
  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=ThermalManagement.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.MassTransfer.Plenum cabin(
    V=186,
    noInitialPressure=false)
    annotation (Placement(transformation(extent={{-52,-42},{-28,-18}})));
  Components.MassTransfer.SourceMassFlow ECSFlow(
    use_in_massFlow=true,
    use_in_T=true,
    use_in_Xw=true)
                   annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-80,-30})));
  Modelica.Blocks.Interfaces.RealInput Xw_fromMix annotation (Placement(
        transformation(extent={{-130,30},{-90,70}}),  iconTransformation(
          extent={{-106,10},{-86,30}})));
  Components.MassTransfer.ValveLin valve(Kv=0.0001) annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={50,-30})));
  Modelica.Blocks.Interfaces.RealInput m_fromMix annotation (Placement(
        transformation(extent={{-130,-30},{-90,10}}),iconTransformation(
          extent={{-106,-70},{-86,-50}})));
  Components.MassTransfer.PressureSink pressureSink
    annotation (Placement(transformation(extent={{76,-42},{100,-18}})));
  Modelica.Blocks.Interfaces.RealInput valveOpening annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={50,44}),  iconTransformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={0,92})));
  Components.MassTransfer.simpleFan simpleFan(
    eta_is=0.7,
    eta_m=0.95,
    omega_nom(displayUnit="rpm") = 314.15926535898,
    volFlow_nom=1,
    Head_nom=10000,
    redeclare
      Components.MassTransfer.FanCharacteristics.FlowCharacteristics.linearFlow
      flowModel)
    annotation (Placement(transformation(extent={{-16,-46},{16,-14}})));
  BoundaryConditions.mechanical mechanical(
    omega(displayUnit="rpm"),
    use_omega=false,
    use_in_omega=true)
    annotation (Placement(transformation(extent={{-12,-8},{12,8}},
        rotation=-90,
        origin={0,0})));
  Modelica.Blocks.Interfaces.RealInput fanSpeed annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-8,66}), iconTransformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={40,92})));
equation
  connect(Q_cabin, cabinHeatFlowRate.Q_flow)
    annotation (Line(points={{-40,100},{-40,72}},
                                              color={0,0,127}));
  connect(cabinHeatFlowRate.port, cabin.thermalPort) annotation (Line(points={{-40,48},
          {-40,-19.2}},                  color={191,0,0}));
  connect(m_fromMix, ECSFlow.in_massFlow)
    annotation (Line(points={{-110,-10},{-91.2,-10},{-91.2,-20.2}},
                                                           color={0,0,127}));
  connect(T_fromMix, ECSFlow.in_T)
    annotation (Line(points={{-110,20},{-82.8,20},{-82.8,-20.2}},
                                                           color={0,0,127}));
  connect(valveOpening, valve.opening)
    annotation (Line(points={{50,44},{50,-23.52}}, color={0,0,127}));
  connect(mechanical.mechanical, simpleFan.shaft)
    annotation (Line(points={{0,-4},{0,-14}},color={135,135,135}));
  connect(mechanical.in_omega, fanSpeed) annotation (Line(points={{-8,14.4},{
          -8,66}},           color={0,0,127}));
  connect(ECSFlow.outlet, cabin.inlet)
    annotation (Line(points={{-66,-30},{-52,-30}}, color={0,0,0}));
  connect(cabin.outlet, simpleFan.inlet)
    annotation (Line(points={{-28,-30},{-16,-30}}, color={0,0,0}));
  connect(valve.outlet, pressureSink.inlet)
    annotation (Line(points={{62,-30},{76,-30}}, color={0,0,0}));
  connect(Xw_fromMix, ECSFlow.in_Xw) annotation (Line(points={{-110,50},{-74,
          50},{-74,-20.2},{-74.4,-20.2}}, color={0,0,127}));
  connect(simpleFan.outlet, valve.inlet)
    annotation (Line(points={{16,-30},{38,-30}}, color={0,0,0}));
  annotation (Documentation(info="<html>

</html>"), experiment(StopTime=5000, Interval=0.1));
end test_fan_sys;
