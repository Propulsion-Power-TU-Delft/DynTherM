within DynTherM.Tests.MassTransfer;
model CircularPipe "Simple test of component pipe"
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.EGW(X=0.1)
    "Refrigerant";

  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  BoundaryConditions.ZeroDimensional.flow_source ECSFlow(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,-30})));
  BoundaryConditions.ZeroDimensional.pressure_sink pressureSink(redeclare
      package Medium = Refrigerant)
    annotation (Placement(transformation(extent={{76,-42},{100,-18}})));
  Components.MassTransfer.CircularPipe pipe(
    redeclare package Medium = Refrigerant,
    allowFlowReversal=false,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    L(displayUnit="mm") = 0.4826,
    D(displayUnit="mm") = 0.005,
    Roughness=0.001e-3)
    annotation (Placement(transformation(extent={{-8,-54},{40,-6}})));

  Modelica.Blocks.Sources.Constant m_input(k=0.3)
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  Modelica.Blocks.Sources.Constant T_input(k=273.15 + 50)
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
equation
  connect(ECSFlow.outlet, pipe.inlet)
    annotation (Line(points={{-46,-30},{-8,-30}},  color={0,0,0}));
  connect(pipe.outlet, pressureSink.inlet)
    annotation (Line(points={{40,-30},{76,-30}}, color={0,0,0}));
  connect(m_input.y, ECSFlow.in_massFlow) annotation (Line(points={{-79,0},{
          -71.2,0},{-71.2,-20.2}}, color={0,0,127}));
  connect(T_input.y, ECSFlow.in_T) annotation (Line(points={{-79,40},{-62.8,40},
          {-62.8,-20.2}}, color={0,0,127}));
  annotation (Documentation(info="<html>

</html>"), experiment(StopTime=5000, Interval=0.1));
end CircularPipe;
