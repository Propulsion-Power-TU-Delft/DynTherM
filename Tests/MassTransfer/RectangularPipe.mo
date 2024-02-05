within DynTherM.Tests.MassTransfer;
model RectangularPipe "Simple test of component pipe"
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1)
    "Refrigerant";

  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  BoundaryConditions.flow_source ECSFlow(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,-30})));
  BoundaryConditions.pressure_sink pressureSink(redeclare package Medium =
        Refrigerant)
    annotation (Placement(transformation(extent={{76,-42},{100,-18}})));
  Components.MassTransfer.RectangularPipe pipe(
    redeclare package Medium = Refrigerant,
    allowFlowReversal=false,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    P_start={20000000000,10000000000},
    T_start={323.15,303.15},
    N_cv=3,
    L(displayUnit="mm") = 0.4826,
    W(displayUnit="mm") = 0.0559,
    H(displayUnit="mm") = 0.00196)
    annotation (Placement(transformation(extent={{-8,-54},{40,-6}})));

  Modelica.Blocks.Sources.Constant m_input(k=0.3)
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  Modelica.Blocks.Sources.Constant T_input(k=273.15 + 50)
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
  BoundaryConditions.thermal_distributed thermal_distributed(
    Nx=pipe.N_cv,
    Ny=1,
    Q(displayUnit="W") = -3e3*ones(pipe.N_cv, 1))
    annotation (Placement(transformation(extent={{-4,-12},{36,10}})));
equation
  connect(ECSFlow.outlet, pipe.inlet)
    annotation (Line(points={{-46,-30},{-8,-30}},  color={0,0,0}));
  connect(pipe.outlet, pressureSink.inlet)
    annotation (Line(points={{40,-30},{76,-30}}, color={0,0,0}));
  connect(m_input.y, ECSFlow.in_massFlow) annotation (Line(points={{-79,0},{
          -71.2,0},{-71.2,-20.2}}, color={0,0,127}));
  connect(T_input.y, ECSFlow.in_T) annotation (Line(points={{-79,40},{-62.8,40},
          {-62.8,-20.2}}, color={0,0,127}));
  connect(thermal_distributed.thermal, pipe.thermalPort) annotation (Line(
        points={{16,-1},{16,-9.74},{16,-9.74},{16,-18.48}}, color={191,0,0}));
  annotation (Documentation(info="<html>

</html>"), experiment(StopTime=5000, Interval=0.1));
end RectangularPipe;
