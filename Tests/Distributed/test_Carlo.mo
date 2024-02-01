within DynTherM.Tests.Distributed;
model test_Carlo
  package Coolant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.5) "Coolant";
  Components.OneDimensional.CircularChannel1D circularChannel1D(
    redeclare model Mat = DynTherM.Materials.Aluminium,
    redeclare package Medium = Coolant,
    L=18,
    R_ext(displayUnit="mm") = 0.0055,
    R_int(displayUnit="mm") = 0.005,
    T_start_solid=298.15,
    T_start_fluid=298.15,
    P_start=400000,
    m_flow_start=7,
    N=1,
    N_channels=50)
    annotation (Placement(transformation(extent={{-32,-72},{32,-8}})));
  BoundaryConditions.flow_source          flow_source(
    redeclare package Medium = Coolant,
    T_nom=298.15,
    massFlow_nom=8,
    use_in_massFlow=false,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-84,-52},{-60,-28}})));
  BoundaryConditions.pressure_sink          pressure_sink(
    redeclare package Medium = Coolant,
    use_ambient=false,
    P_di=400000)
    annotation (Placement(transformation(extent={{64,-52},{88,-28}})));
  Components.HeatTransfer.HeatCapacity          battery_pack(T_start=298.15, C=
        1.75e7)
    annotation (Placement(transformation(extent={{-18,38},{18,74}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier          heatFlowMultiplier(Nx=1,
                    Ny=1)
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=180,
        origin={0,12})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow internal_heat_generation
    annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=90,
        origin={40,44})));
  Modelica.Blocks.Sources.Constant const(k=350e3)
    annotation (Placement(transformation(extent={{68,72},{52,88}})));
  inner Components.Environment          environment(
    use_di_altitude=true,
    use_di_Mach_inf=true,
    use_in_altitude=false,
    use_in_V_inf=false,
    use_ext_sw=false,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-80,52},{-46,86}})));
equation
  connect(battery_pack.port, heatFlowMultiplier.single) annotation (Line(points=
         {{0,38},{0,31},{1.55431e-15,31},{1.55431e-15,24}}, color={191,0,0}));
  connect(heatFlowMultiplier.distributed, circularChannel1D.solid_surface)
    annotation (Line(points={{-1.77636e-15,0},{-1.77636e-15,0.785},{0,0.785},{0,
          -24.96}}, color={191,0,0}));
  connect(flow_source.outlet, circularChannel1D.inlet)
    annotation (Line(points={{-60,-40},{-32,-40}}, color={0,0,0}));
  connect(circularChannel1D.outlet, pressure_sink.inlet)
    annotation (Line(points={{32,-40},{64,-40}}, color={0,0,0}));
  connect(heatFlowMultiplier.single, internal_heat_generation.port)
    annotation (Line(points={{0,24},{40,24},{40,32}}, color={191,0,0}));
  connect(const.y, internal_heat_generation.Q_flow)
    annotation (Line(points={{51.2,80},{40,80},{40,56}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end test_Carlo;
