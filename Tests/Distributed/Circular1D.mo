within DynTherM.Tests.Distributed;
model Circular1D
  package Refrigerant = Modelica.Media.Water.ConstantPropertyLiquidWater;
  //package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1);
  BoundaryConditions.ZeroDimensional.flow_source flow_source_distributed(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,-14},{-14,14}},
        rotation=180,
        origin={-80,80})));
  BoundaryConditions.ZeroDimensional.pressure_sink pressureSink_distributed(
      redeclare package Medium = Refrigerant)
    annotation (Placement(transformation(extent={{68,68},{92,92}})));
  inner Components.Environment environment(
    allowFlowReversal=false, initOpt=DynTherM.Choices.InitOpt.steadyState)
                                                  annotation (Placement(transformation(extent={{38,20},
            {78,60}})));
  Components.OneDimensional.CircularChannel1D channel1D(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.4826,
    R_ext(displayUnit="mm") = 0.003,
    R_int(displayUnit="mm") = 0.0025,
    V_inertia=1e-10,
    T_start_solid=323.15,
    T_start_fluid=323.15,
    N_cv=3,
    N_channels=1)
         annotation (Placement(transformation(extent={{-28,52},{28,108}})));

  BoundaryConditions.ThreeDimensional.thermal3D thermal_distributed(
    Nx=3,
    Ny=1,
    Q(displayUnit="W") = -3e3*ones(3, 1))
    annotation (Placement(transformation(extent={{-18,98},{18,120}})));
  Components.OneDimensional.CircularCV cv_1(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.001*(482.6/3),
    R_ext(displayUnit="mm") = 0.003,
    R_int(displayUnit="mm") = 0.0025,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{-60,-120},{-20,-80}})));
  Components.OneDimensional.CircularCV cv_2(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.001*(482.6/3),
    R_ext(displayUnit="mm") = 0.003,
    R_int(displayUnit="mm") = 0.0025,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{-20,-120},{20,-80}})));
  Components.OneDimensional.CircularCV cv_3(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.001*(482.6/3),
    R_ext(displayUnit="mm") = 0.003,
    R_int(displayUnit="mm") = 0.0025,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{20,-120},{60,-80}})));
  BoundaryConditions.ZeroDimensional.pressure_sink pressureSink_multiple(
      redeclare package Medium = Refrigerant)
    annotation (Placement(transformation(extent={{68,-112},{92,-88}})));
  BoundaryConditions.ZeroDimensional.flow_source flow_source_multiple(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-80,-100})));
  Modelica.Blocks.Sources.Constant m(k=0.3)
    annotation (Placement(transformation(extent={{-50,-50},{-70,-30}})));
  Modelica.Blocks.Sources.Constant T(k=273.15 + 50)
    annotation (Placement(transformation(extent={{-50,40},{-70,60}})));
  BoundaryConditions.ZeroDimensional.thermal thermal_1(Q(displayUnit="kW") = -3000)
    annotation (Placement(transformation(extent={{-52,-66},{-34,-54}})));
  BoundaryConditions.ZeroDimensional.thermal thermal_3(Q(displayUnit="kW") = -3000)
    annotation (Placement(transformation(extent={{28,-66},{46,-54}})));
  BoundaryConditions.ZeroDimensional.thermal thermal_2(Q(displayUnit="kW") = -3000)
    annotation (Placement(transformation(extent={{-12,-66},{6,-54}})));
  Components.OneDimensional.CircularCV cv(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.4826,
    R_ext(displayUnit="mm") = 0.003,
    R_int(displayUnit="mm") = 0.0025,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
  BoundaryConditions.ZeroDimensional.flow_source flow_source_single(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,0})));
  BoundaryConditions.ZeroDimensional.pressure_sink pressureSink_single(
      redeclare package Medium = Refrigerant)
    annotation (Placement(transformation(extent={{68,-12},{92,12}})));
  BoundaryConditions.ZeroDimensional.thermal thermal(Q(displayUnit="kW") = 1000
      *(-3*3)) annotation (Placement(transformation(extent={{-12,34},{6,46}})));
equation

  connect(flow_source_distributed.outlet, channel1D.inlet)
    annotation (Line(points={{-66,80},{-28,80}}, color={0,0,0}));
  connect(channel1D.outlet, pressureSink_distributed.inlet)
    annotation (Line(points={{28,80},{68,80}}, color={0,0,0}));
  connect(thermal_distributed.thermal, channel1D.solid_surface) annotation (
      Line(points={{-3.55271e-15,109},{-3.55271e-15,103.55},{3.55271e-15,103.55},
          {3.55271e-15,93.16}},
        color={191,0,0}));
  connect(cv_1.outlet, cv_2.inlet)
    annotation (Line(points={{-24,-100},{-16,-100}}, color={0,0,0}));
  connect(cv_2.outlet, cv_3.inlet)
    annotation (Line(points={{16,-100},{24,-100}}, color={0,0,0}));
  connect(flow_source_multiple.outlet, cv_1.inlet)
    annotation (Line(points={{-66,-100},{-56,-100}}, color={0,0,0}));
  connect(cv_3.outlet, pressureSink_multiple.inlet)
    annotation (Line(points={{56,-100},{68,-100}}, color={0,0,0}));
  connect(T.y, flow_source_distributed.in_T) annotation (Line(points={{-71,50},
          {-82.8,50},{-82.8,70.2}}, color={0,0,127}));
  connect(T.y, flow_source_multiple.in_T) annotation (Line(points={{-71,50},{
          -82.8,50},{-82.8,-90.2}}, color={0,0,127}));
  connect(m.y, flow_source_distributed.in_massFlow) annotation (Line(points={{
          -71,-40},{-91.2,-40},{-91.2,70.2}}, color={0,0,127}));
  connect(m.y, flow_source_multiple.in_massFlow) annotation (Line(points={{-71,
          -40},{-91.2,-40},{-91.2,-90.2}}, color={0,0,127}));
  connect(thermal_1.thermal, cv_1.solid_surface)
    annotation (Line(points={{-40,-60},{-40,-84}}, color={191,0,0}));
  connect(thermal_2.thermal, cv_2.solid_surface)
    annotation (Line(points={{0,-60},{0,-84}}, color={191,0,0}));
  connect(thermal_3.thermal, cv_3.solid_surface)
    annotation (Line(points={{40,-60},{40,-84}}, color={191,0,0}));
  connect(flow_source_single.outlet, cv.inlet) annotation (Line(points={{-46,
          -3.44169e-15},{-31,-3.44169e-15},{-31,0},{-16,0}}, color={0,0,0}));
  connect(cv.outlet, pressureSink_single.inlet)
    annotation (Line(points={{16,0},{68,0}}, color={0,0,0}));
  connect(thermal.thermal, cv.solid_surface)
    annotation (Line(points={{0,40},{0,16}}, color={191,0,0}));
  connect(T.y, flow_source_single.in_T) annotation (Line(points={{-71,50},{-78,
          50},{-78,20},{-62.8,20},{-62.8,9.8}}, color={0,0,127}));
  connect(m.y, flow_source_single.in_massFlow) annotation (Line(points={{-71,
          -40},{-78,-40},{-78,16},{-71.2,16},{-71.2,9.8}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -120},{100,120}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{100,
            120}})));
end Circular1D;
