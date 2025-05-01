within DynTherM.Tests.Battery;
model PouchCellModuleManual1D_test

  PouchCellModuleManual1D module(
    W_cell(displayUnit="mm") = 0.35,
    H_cell(displayUnit="mm") = 0.1,
    t_cell(displayUnit="mm") = 0.01,
    t_fw_int(displayUnit="mm") = 0.001,
    t_fw_ext(displayUnit="mm") = 0.001,
    t_gap(displayUnit="mm") = 0,
    t_resin(displayUnit="mm") = 0.0005,
    t_frame(displayUnit="mm") = 0.005,
    C_nom=230400,
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    SoC_start=0.1,
    Tstart=298.15,
    N_cv=3) annotation (Placement(transformation(extent={{-40,-20},{40,20}})));
  BoundaryConditions.OneDimensional.thermal1D heat_source_left(
    Nx=3,
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-21,12},{21,-12}},
        rotation=-90,
        origin={-60,43})));
  Modelica.Blocks.Sources.Step input_left(
    height=-10,
    offset=10,
    startTime=30)
    annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
  Modelica.Blocks.Sources.Step input_bottom(height=10, startTime=30)
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  BoundaryConditions.OneDimensional.thermal1D heat_source_bottom(
    Nx=3,
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-21,12},{21,-12}},
        rotation=0,
        origin={-30,-39})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-28,24},{-12,40}})));
  Components.OneDimensional.ConvectionRadiation1D convectionRadiation_top(
    A=module.frame_top.A,
    N=3)
    annotation (Placement(transformation(extent={{-10,40},{10,60}})));
  Components.OneDimensional.ConvectionRadiation1D convectionRadiation_bottom(
    A=module.frame_bottom.A,
    N=3)
    annotation (Placement(transformation(extent={{-10,-60},{10,-80}})));
  Components.OneDimensional.ConvectionRadiation1D convectionRadiation_right(
    A=module.frame_right.A,
    N=3)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={70,0})));
  Components.OneDimensional.ConvectionRadiation1D convectionRadiation_left(
    A=module.frame_left.A,
    N=3)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=-90,
        origin={-70,0})));
  BoundaryConditions.OneDimensional.thermal1D West_BC(
    Nx=3,
    T(displayUnit="K") = 298.15*ones(3),
    use_di_Q=false,
    use_di_T=true,
    use_in_Q=false) annotation (Placement(transformation(
        extent={{-22,12},{22,-12}},
        rotation=-90,
        origin={-100,0})));
  BoundaryConditions.OneDimensional.thermal1D East_BC(
    Nx=3,
    T(displayUnit="K") = 298.15*ones(3),
    use_di_Q=false,
    use_di_T=true,
    use_in_Q=false) annotation (Placement(transformation(
        extent={{-22,-12},{22,12}},
        rotation=-90,
        origin={100,0})));
  BoundaryConditions.OneDimensional.thermal1D South_BC(
    Nx=3,
    T(displayUnit="K") = 298.15*ones(3),
    use_di_Q=false,
    use_di_T=true,
    use_in_Q=false) annotation (Placement(transformation(
        extent={{-22,12},{22,-12}},
        rotation=0,
        origin={0,-98})));
  BoundaryConditions.OneDimensional.thermal1D North_BC(
    Nx=3,
    T(displayUnit="K") = 298.15*ones(3),
    use_di_Q=false,
    use_di_T=true,
    use_in_Q=false) annotation (Placement(transformation(
        extent={{-22,12},{22,-12}},
        rotation=0,
        origin={0,98})));
equation
  connect(input_left.y,heat_source_left. in_Q) annotation (Line(points={{-79,50},
          {-64.8,50}},                       color={0,0,127}));
  connect(input_bottom.y, heat_source_bottom.in_Q) annotation (Line(points={{-59,
          -70},{-37,-70},{-37,-43.8}}, color={0,0,127}));
  connect(heat_source_left.thermal, module.West) annotation (Line(points={{-60,
          43},{-50,43},{-50,-3.33333},{-40,-3.33333}}, color={191,0,0}));
  connect(heat_source_bottom.thermal, module.South) annotation (Line(points={{-30,
          -39},{-30,-30},{0,-30},{0,-20}}, color={191,0,0}));
  connect(module.p, ground.p) annotation (Line(points={{-40,10},{-46,10},{-46,
          40},{-20,40}}, color={0,0,255}));
  connect(convectionRadiation_left.inlet, module.West) annotation (Line(points=
          {{-60,0},{-50,0},{-50,-3.33333},{-40,-3.33333}}, color={191,0,0}));
  connect(convectionRadiation_right.inlet, module.East) annotation (Line(points
        ={{60,0},{50,0},{50,-3.33333},{40,-3.33333}}, color={191,0,0}));
  connect(convectionRadiation_bottom.inlet, module.South)
    annotation (Line(points={{0,-60},{0,-20}}, color={191,0,0}));
  connect(module.North, convectionRadiation_top.inlet)
    annotation (Line(points={{0,20},{0,40}}, color={191,0,0}));
  connect(South_BC.thermal, convectionRadiation_bottom.outlet)
    annotation (Line(points={{0,-98},{0,-80}}, color={191,0,0}));
  connect(West_BC.thermal, convectionRadiation_left.outlet)
    annotation (Line(points={{-100,0},{-80,0}}, color={191,0,0}));
  connect(convectionRadiation_right.outlet, East_BC.thermal)
    annotation (Line(points={{80,0},{100,0}}, color={191,0,0}));
  connect(convectionRadiation_top.outlet, North_BC.thermal)
    annotation (Line(points={{0,60},{0,98}}, color={191,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=60, __Dymola_Algorithm="Dassl"));
end PouchCellModuleManual1D_test;
