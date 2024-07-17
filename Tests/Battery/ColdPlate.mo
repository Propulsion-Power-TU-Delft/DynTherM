within DynTherM.Tests.Battery;
model ColdPlate
  package Water = Modelica.Media.Water.ConstantPropertyLiquidWater;

  parameter Integer N_longitudinal=10;
  parameter Integer N_transversal=15;

  Components.OneDimensional.ParallelRectangularColdPlate
    parallelRectangularColdPlate(
    redeclare package Medium = Water,
    L=0.5,
    W_plate=0.35,
    allowFlowReversal=environment.allowFlowReversal,
    initOpt=environment.initOpt,
    T_start_plate=298.15,
    T_start_fluid=298.15,
    P_start=400000,
    m_flow_start=0.5,
    rho_start(displayUnit="kg/m3") = 1000,
    Re_start=4000,
    Pr_start=7,
    N_cv=N_longitudinal)
    annotation (Placement(transformation(extent={{-24,-16},{24,-64}})));
  Components.OneDimensional.ParallelCircularColdPlate parallelCircularColdPlate(
    redeclare package Medium = Water,
    L=0.5,
    W_plate=0.35,
    H_plate(displayUnit="mm"),
    R_int(displayUnit="mm") = 0.0007,
    allowFlowReversal=environment.allowFlowReversal,
    initOpt=environment.initOpt,
    T_start_plate=298.15,
    T_start_fluid=298.15,
    P_start=400000,
    m_flow_start=0.5,
    rho_start(displayUnit="kg/m3") = 1000,
    Pr_start=7,
    N_cv=N_longitudinal,
    N_channels=N_transversal)
    annotation (Placement(transformation(extent={{-24,16},{24,64}})));
  BoundaryConditions.flow_source flow_source(
    redeclare package Medium = Water,
    P_nom=400000,
    T_nom=298.15,
    massFlow_nom=0.5,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-80,-52},{-56,-28}})));
  BoundaryConditions.flow_source flow_source1(
    redeclare package Medium = Water,
    P_nom=400000,
    T_nom=298.15,
    massFlow_nom=0.5,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-80,28},{-56,52}})));
  BoundaryConditions.pressure_sink pressure_sink(
    redeclare package Medium = Water,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=100000,
    T_di=298.15)
    annotation (Placement(transformation(extent={{60,30},{80,50}})));
  BoundaryConditions.pressure_sink pressure_sink1(
    redeclare package Medium = Water,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=100000,
    T_di=298.15)
    annotation (Placement(transformation(extent={{60,-50},{80,-30}})));
  BoundaryConditions.thermal_distributed thermal_distributed(Nx=N_longitudinal,
      Ny=1)
    annotation (Placement(transformation(extent={{-18,-90},{18,-70}})));
  BoundaryConditions.thermal_distributed thermal_distributed1(Nx=N_longitudinal,
      Ny=N_transversal)
    annotation (Placement(transformation(extent={{-18,70},{18,90}})));
  inner Components.Environment environment(allowFlowReversal=false, initOpt=
        DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{58,58},{96,96}})));
equation
  connect(flow_source1.outlet, parallelCircularColdPlate.inlet)
    annotation (Line(points={{-56,40},{-24,40}}, color={0,0,0}));
  connect(flow_source.outlet, parallelRectangularColdPlate.inlet)
    annotation (Line(points={{-56,-40},{-24,-40}}, color={0,0,0}));
  connect(parallelCircularColdPlate.outlet, pressure_sink.inlet)
    annotation (Line(points={{24,40},{60,40}}, color={0,0,0}));
  connect(parallelRectangularColdPlate.outlet, pressure_sink1.inlet)
    annotation (Line(points={{24,-40},{60,-40}}, color={0,0,0}));
  connect(thermal_distributed1.thermal, parallelCircularColdPlate.upper_surface)
    annotation (Line(points={{0,80},{0,56.32}}, color={191,0,0}));
  connect(parallelRectangularColdPlate.upper_surface, thermal_distributed.thermal)
    annotation (Line(points={{0,-56.32},{0,-80}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ColdPlate;
