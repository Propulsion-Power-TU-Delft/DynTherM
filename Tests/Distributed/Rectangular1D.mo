within DynTherM.Tests.Distributed;
model Rectangular1D
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1)
    "Refrigerant";
  BoundaryConditions.flow_source flow_source_distributed(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,-14},{-14,14}},
        rotation=180,
        origin={-80,80})));
  BoundaryConditions.pressure_sink pressureSink_distributed(redeclare package
      Medium = Refrigerant)
    annotation (Placement(transformation(extent={{68,68},{92,92}})));
  inner Components.Environment environment(
    allowFlowReversal=false, initOpt=DynTherM.Choices.InitOpt.steadyState)
                                                  annotation (Placement(transformation(extent={{38,20},
            {78,60}})));
  Components.OneDimensional.RectangularChannel1D channel1D(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.4826,
    W(displayUnit="mm") = 0.0559,
    H(displayUnit="mm") = 0.00196,
    t_north(displayUnit="mm") = 0.001,
    t_east(displayUnit="mm") = 0.001,
    t_south(displayUnit="mm") = 0.001,
    t_west(displayUnit="mm") = 0.001,
    T_start_solid=323.15,
    T_start_fluid=323.15,
    N=3,
    N_channels=1)
    annotation (Placement(transformation(extent={{-28,52},{28,108}})));

  BoundaryConditions.thermal_flux_distributed thermal_distributed(
    Nx=3,
    Ny=1,
    phi=-1e6*ones(3, 1))
    annotation (Placement(transformation(extent={{-40,100},{-2,122}})));
  Components.OneDimensional.RectangularCV cv_1(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.001*(482.6/3),
    W(displayUnit="mm") = 0.0559,
    H(displayUnit="mm") = 0.00196,
    t_north(displayUnit="mm") = 0.001,
    t_east(displayUnit="mm") = 0.001,
    t_south(displayUnit="mm") = 0.001,
    t_west(displayUnit="mm") = 0.001,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{-60,-120},{-20,-80}})));
  Components.OneDimensional.RectangularCV cv_2(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.001*(482.6/3),
    W(displayUnit="mm") = 0.0559,
    H(displayUnit="mm") = 0.00196,
    t_north(displayUnit="mm") = 0.001,
    t_east(displayUnit="mm") = 0.001,
    t_south(displayUnit="mm") = 0.001,
    t_west(displayUnit="mm") = 0.001,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{-20,-120},{20,-80}})));
  Components.OneDimensional.RectangularCV cv_3(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.001*(482.6/3),
    W(displayUnit="mm") = 0.0559,
    H(displayUnit="mm") = 0.00196,
    t_north(displayUnit="mm") = 0.001,
    t_east(displayUnit="mm") = 0.001,
    t_south(displayUnit="mm") = 0.001,
    t_west(displayUnit="mm") = 0.001,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{20,-120},{60,-80}})));
  BoundaryConditions.pressure_sink pressureSink_multiple(redeclare package
      Medium = Refrigerant)
    annotation (Placement(transformation(extent={{68,-112},{92,-88}})));
  BoundaryConditions.flow_source flow_source_multiple(
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
  BoundaryConditions.thermal_flux thermal_1(phi=-1e6)
    annotation (Placement(transformation(extent={{-64,-76},{-46,-64}})));
  BoundaryConditions.thermal_flux thermal_3(phi=-1e6)
    annotation (Placement(transformation(extent={{16,-76},{34,-64}})));
  BoundaryConditions.thermal_flux thermal_2(phi=-1e6)
    annotation (Placement(transformation(extent={{-24,-76},{-6,-64}})));
  Components.OneDimensional.RectangularCV cv(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.4826,
    W(displayUnit="mm") = 0.0559,
    H(displayUnit="mm") = 0.00196,
    t_north(displayUnit="mm") = 0.001,
    t_east(displayUnit="mm") = 0.001,
    t_south(displayUnit="mm") = 0.001,
    t_west(displayUnit="mm") = 0.001,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
  BoundaryConditions.flow_source flow_source_single(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,0})));
  BoundaryConditions.pressure_sink pressureSink_single(
    redeclare package Medium = Refrigerant)
    annotation (Placement(transformation(extent={{68,-12},{92,12}})));
  BoundaryConditions.thermal_flux thermal(phi=-1e6)
    annotation (Placement(transformation(extent={{-24,24},{-6,36}})));
equation

  connect(flow_source_distributed.outlet, channel1D.inlet)
    annotation (Line(points={{-66,80},{-28,80}}, color={0,0,0}));
  connect(channel1D.outlet, pressureSink_distributed.inlet)
    annotation (Line(points={{28,80},{68,80}}, color={0,0,0}));
  connect(cv_1.outlet, cv_2.inlet)
    annotation (Line(points={{-24,-100},{-16,-100}}, color={0,0,0}));
  connect(cv_2.outlet, cv_3.inlet)
    annotation (Line(points={{16,-100},{24,-100}}, color={0,0,0}));
  connect(flow_source_multiple.outlet, cv_1.inlet)
    annotation (Line(points={{-66,-100},{-56,-100}}, color={0,0,0}));
  connect(cv_3.outlet, pressureSink_multiple.inlet)
    annotation (Line(points={{56,-100},{68,-100}}, color={0,0,0}));
  connect(T.y, flow_source_distributed.in_T) annotation (Line(points={{-71,50},{
          -82.8,50},{-82.8,70.2}}, color={0,0,127}));
  connect(T.y, flow_source_multiple.in_T) annotation (Line(points={{-71,50},{-82.8,
          50},{-82.8,-90.2}}, color={0,0,127}));
  connect(m.y, flow_source_distributed.in_massFlow) annotation (Line(points={{-71,
          -40},{-91.2,-40},{-91.2,70.2}}, color={0,0,127}));
  connect(m.y, flow_source_multiple.in_massFlow) annotation (Line(points={{-71,-40},
          {-91.2,-40},{-91.2,-90.2}}, color={0,0,127}));
  connect(flow_source_single.outlet, cv.inlet) annotation (Line(points={{-46,-3.44169e-15},
          {-31,-3.44169e-15},{-31,0},{-16,0}}, color={0,0,0}));
  connect(cv.outlet, pressureSink_single.inlet)
    annotation (Line(points={{16,0},{68,0}}, color={0,0,0}));
  connect(T.y, flow_source_single.in_T) annotation (Line(points={{-71,50},{-78,50},
          {-78,20},{-62.8,20},{-62.8,9.8}}, color={0,0,127}));
  connect(m.y, flow_source_single.in_massFlow) annotation (Line(points={{-71,-40},
          {-78,-40},{-78,16},{-71.2,16},{-71.2,9.8}}, color={0,0,127}));
  connect(thermal_distributed.thermal_flux, channel1D.solid_surface_north)
    annotation (Line(points={{-21,111},{-21,92.88}}, color={255,127,0}));
  connect(thermal.thermal_flux, cv.solid_surface_north)
    annotation (Line(points={{-12,30},{-12,14}}, color={255,127,0}));
  connect(thermal_1.thermal_flux, cv_1.solid_surface_north)
    annotation (Line(points={{-52,-70},{-52,-86}}, color={255,127,0}));
  connect(thermal_2.thermal_flux, cv_2.solid_surface_north)
    annotation (Line(points={{-12,-70},{-12,-86}}, color={255,127,0}));
  connect(thermal_3.thermal_flux, cv_3.solid_surface_north)
    annotation (Line(points={{28,-70},{28,-86}}, color={255,127,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},
            {100,120}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{100,120}})));
end Rectangular1D;
