within DynTherM.Tests.Battery;
model PouchCellModuleCooling
  //package Water = Modelica.Media.Water.StandardWater;
  package Water = Modelica.Media.Water.ConstantPropertyLiquidWater;

  parameter Integer N_cv=10;
  parameter Length W_cell=0.35;
  parameter Length H_cell=0.1;
  parameter Length t_cell=0.01;
  parameter Length t_fw=0.001;

  final parameter Length L_plate=battery_module.N_cells*t_cell +
    (battery_module.N_cells - 1)*t_fw;

  Modelica.Blocks.Sources.TimeTable I_charging(table=[0,200; 200,200; 200,200; 500,
        200; 501,120; 700,120; 701,100; 900,100; 901,120; 1000,120; 1001,70; 1200,
        70]) annotation (Placement(transformation(extent={{118,52},{102,68}})));
  inner Components.Environment environment(allowFlowReversal=false, initOpt=
        DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-86,34},{-42,78}})));
  Systems.Battery.PouchModuleFirewall battery_module(
    W_cell=W_cell,
    H_cell=H_cell,
    t_cell(displayUnit="mm") = t_cell,
    t_fw(displayUnit="mm") = t_fw,
    C_nom(displayUnit="Ah") = 230400,
    initOpt=environment.initOpt,
    SoC_start=0.1,
    Tstart=298.15,
    N_cv=N_cv,
    N_parallel=1,
    N_series=10)
    annotation (Placement(transformation(extent={{-32,-14},{48,66}})));
  Components.Battery.ParallelRectangularColdPlate parallelRectangularColdPlate(
    redeclare package Medium = Water,
    L=L_plate,
    W_plate=W_cell,
      allowFlowReversal=environment.allowFlowReversal,
    initOpt=environment.initOpt,
    T_start_plate=298.15,
    T_start_fluid=298.15,
    P_start=400000,
    m_flow_start=0.5,
    rho_start(displayUnit="kg/m3") = 1000,
    Re_start=4000,
    Pr_start=7,                                        N_cv=battery_module.N_cells)
    annotation (Placement(transformation(extent={{-24,-54},{24,-6}})));
  BoundaryConditions.flow_source flow_source(
    redeclare package Medium = Water,
    P_nom=400000,
    T_nom=298.15,
    massFlow_nom=0.5,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-84,-42},{-60,-18}})));
  BoundaryConditions.pressure_sink pressure_sink(
    redeclare package Medium = Water,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=100000,
    T_di=298.15)
    annotation (Placement(transformation(extent={{70,-40},{90,-20}})));
equation
  connect(I_charging.y, battery_module.I)
    annotation (Line(points={{101.2,60},{4.8,60},{4.8,45.2}},
                                                        color={0,0,127}));
  connect(parallelRectangularColdPlate.upper_surface, battery_module.Bottom)
    annotation (Line(points={{0,-13.68},{0,-0.4}}, color={191,0,0}));
  connect(flow_source.outlet, parallelRectangularColdPlate.inlet)
    annotation (Line(points={{-60,-30},{-24,-30}}, color={0,0,0}));
  connect(parallelRectangularColdPlate.outlet, pressure_sink.inlet)
    annotation (Line(points={{24,-30},{70,-30}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{120,80}})),                                    Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{120,
            80}})),
    experiment(
      StopTime=1200,
      Interval=1,
      __Dymola_Algorithm="Dassl"));
end PouchCellModuleCooling;
