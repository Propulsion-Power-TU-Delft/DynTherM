within DynTherM.Tests.Battery;
model TestPolestar
  Systems.Battery.PouchModuleFirewall battery_module(
    redeclare model InPlaneCellMat = DynTherM.Materials.PolestarCellInPlane,
    W_cell=0.35,
    H_cell=0.1,
    t_cell(displayUnit="mm") = 0.01,
    t_fw(displayUnit="mm") = 0.001,
    C_nom(displayUnit="Ah") = 239040,
    initOpt=environment.initOpt,
    SoC_start=0.1,
    Tstart=298.15,
    N_cv=12,
    N_parallel=3,
    N_series=4)
    annotation (Placement(transformation(extent={{0,-10},{80,70}})));
  inner Components.Environment environment(allowFlowReversal=false, initOpt=
        DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-102,62},{-58,106}})));
  Modelica.Blocks.Sources.TimeTable I_charging(table=[0,600; 200,600; 200,600;
        500,600; 501,360; 700,360; 701,300; 900,300; 901,360; 1000,360; 1001,
        210; 1200,210])
             annotation (Placement(transformation(extent={{100,72},{84,88}})));
  Components.Battery.ColdPlatePolestar coldPlatePolestar(
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    L=0.35,
    t(displayUnit="mm") = 0.012,
    d(displayUnit="mm") = 0.02,
    R_int(displayUnit="mm") = 0.005,
    T_start_solid=298.15,
    T_start_fluid=298.15,
    N_cv=12,
    N_channels=1)
    annotation (Placement(transformation(extent={{-34,-106},{58,-36}})));
  BoundaryConditions.pressure_sink pressure_sink(
    redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=100000,
    T_di=298.15)
    annotation (Placement(transformation(extent={{-56,-94},{-76,-74}})));

  BoundaryConditions.flow_source flow_source(
    redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
    P_nom=100000,
    T_nom=298.15,
    massFlow_nom=0.04416,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-78,-66},{-54,-42}})));

  Components.OneDimensional.WallConduction1D wallConduction1D(
    t=0.64,
    A=0.050750,
    Tstart=298.15,
    N=12) annotation (Placement(transformation(extent={{-2,-32},{32,-8}})));
equation
  connect(battery_module.I,I_charging. y) annotation (Line(points={{36.8,49.2},
          {36.8,80},{83.2,80}},                           color={0,0,127}));
  connect(pressure_sink.inlet, coldPlatePolestar.outlet) annotation (Line(
        points={{-56,-84},{-40,-84},{-40,-78.7},{-30.7143,-78.7}},   color={0,0,
          0}));
  connect(flow_source.outlet, coldPlatePolestar.inlet) annotation (Line(points={{-54,-54},
          {-40,-54},{-40,-70.3},{-30.0571,-70.3}},              color={0,0,0}));
  connect(battery_module.Bottom, wallConduction1D.inlet) annotation (Line(
        points={{32,3.6},{32,-8},{15,-8},{15,-16.4}}, color={191,0,0}));
  connect(coldPlatePolestar.Top, wallConduction1D.outlet) annotation (Line(
        points={{15.2857,-54.9},{15,-52},{15,-23.6}},               color={191,
          0,0}));

 // assert((battery_module.N_cv * battery_module.t_cell) >= 6*coldPlatePolestar.d, "Thickness of the plate greater than channel diameter", AssertionLevel.warning);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end TestPolestar;
