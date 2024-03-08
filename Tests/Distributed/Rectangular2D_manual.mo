within DynTherM.Tests.Distributed;
model Rectangular2D_manual
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.5)
    "Refrigerant";

  parameter Integer N=3 "Number of longitudinal sections in which the tube is discretized";
  parameter Modelica.Units.SI.Length L "Channel length";
  parameter Modelica.Units.SI.Length W "Width of the control volume";
  parameter Modelica.Units.SI.Length H "Height of the control volume";
  parameter Modelica.Units.SI.Length t_ext "Thickness of external walls";
  parameter Modelica.Units.SI.Length t_int "Thickness of internal walls";

  parameter Modelica.Units.SI.HeatFlux phi;
  parameter Modelica.Units.SI.Temperature T_start_solid
    "Temperature of solid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature T_start_fluid
    "Temperature of fluid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure P_start=101325
    "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.MassFraction X_start[Refrigerant.nX]=Refrigerant.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Refrigerant.ThermodynamicState state_start = Refrigerant.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));

  BoundaryConditions.flow_source west_flow(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-80,-60})));
  Modelica.Blocks.Interfaces.RealInput T annotation (Placement(transformation(
          extent={{-130,10},{-90,50}}), iconTransformation(extent={{-106,-30},{-86,
            -10}})));
  Modelica.Blocks.Interfaces.RealInput m annotation (Placement(transformation(
          extent={{-130,-50},{-90,-10}}), iconTransformation(extent={{-106,-70},
            {-86,-50}})));
  BoundaryConditions.pressure_sink pressureSink(redeclare package Medium =
        Refrigerant)
    annotation (Placement(transformation(extent={{68,-12},{92,12}})));
  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.steadyState) annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.OneDimensional.RectangularChannelFlux1D west(
    redeclare package Medium = Refrigerant,
    L=L,
    H=H,
    W=W,
    t_north=t_ext,
    t_south=t_ext,
    t_east=t_int,
    t_west=t_int,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    N=N) annotation (Placement(transformation(extent={{-30,-90},{30,-30}})));

  BoundaryConditions.thermal_flux_distributed thermal_distributed(
    N=N,
    phi=phi*ones(N))
    annotation (Placement(transformation(extent={{-78,78},{-42,100}})));
  Components.OneDimensional.RectangularChannelFlux1D internal(
    redeclare package Medium = Refrigerant,
    L=L,
    H=H,
    W=W,
    t_north=t_ext,
    t_south=t_ext,
    t_east=t_int,
    t_west=t_int,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    N=N) annotation (Placement(transformation(extent={{-30,-30},{30,30}})));
  Components.OneDimensional.RectangularChannelFlux1D east(
    redeclare package Medium = Refrigerant,
    L=L,
    H=H,
    W=W,
    t_north=t_ext,
    t_east=t_int,
    t_south=t_ext,
    t_west=t_int,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    N=N) annotation (Placement(transformation(extent={{-30,30},{30,90}})));
  BoundaryConditions.flow_source internal_flow(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-82,0})));
  BoundaryConditions.flow_source east_flow(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-82,60})));
equation

  connect(internal.outlet, pressureSink.inlet)
    annotation (Line(points={{30,0},{68,0}}, color={0,0,0}));
  connect(west.outlet, pressureSink.inlet)
    annotation (Line(points={{30,-60},{52,-60},{52,0},{68,0}}, color={0,0,0}));
  connect(east.outlet, pressureSink.inlet)
    annotation (Line(points={{30,60},{52,60},{52,0},{68,0}}, color={0,0,0}));
  connect(east.solid_surface_north, thermal_distributed.thermal_flux)
    annotation (Line(points={{-22.5,73.8},{-22.5,74},{-60,74},{-60,89}}, color={
          255,127,0}));
  connect(internal.solid_surface_north, thermal_distributed.thermal_flux)
    annotation (Line(points={{-22.5,13.8},{-22.5,14},{-60,14},{-60,89}}, color={
          255,127,0}));
  connect(west.solid_surface_north, thermal_distributed.thermal_flux)
    annotation (Line(points={{-22.5,-46.2},{-60,-46.2},{-60,89}}, color={255,127,
          0}));
  connect(east.solid_surface_west, internal.solid_surface_east) annotation (
      Line(points={{22.5,73.8},{22.5,30},{-7.5,30},{-7.5,13.8}}, color={255,127,
          0}));
  connect(internal.solid_surface_west, west.solid_surface_east) annotation (
      Line(points={{22.5,13.8},{22.5,-30},{-7.5,-30},{-7.5,-46.2}}, color={255,127,
          0}));
  connect(east_flow.outlet, east.inlet)
    annotation (Line(points={{-68,60},{-30,60}}, color={0,0,0}));
  connect(internal_flow.outlet, internal.inlet) annotation (Line(points={{-68,-3.44169e-15},
          {-49,-3.44169e-15},{-49,0},{-30,0}}, color={0,0,0}));
  connect(west_flow.outlet, west.inlet)
    annotation (Line(points={{-66,-60},{-30,-60}}, color={0,0,0}));
  connect(m, west_flow.in_massFlow) annotation (Line(points={{-110,-30},{-91.2,-30},
          {-91.2,-50.2}}, color={0,0,127}));
  connect(m, internal_flow.in_massFlow) annotation (Line(points={{-110,-30},{-92,
          -30},{-92,9.8},{-93.2,9.8}}, color={0,0,127}));
  connect(m, east_flow.in_massFlow) annotation (Line(points={{-110,-30},{-92,-30},
          {-92,69.8},{-93.2,69.8}}, color={0,0,127}));
  connect(T, internal_flow.in_T) annotation (Line(points={{-110,30},{-84.8,30},{
          -84.8,9.8}}, color={0,0,127}));
  connect(T, east_flow.in_T) annotation (Line(points={{-110,30},{-64,30},{-64,54},
          {-62,54},{-62,74},{-64,74},{-64,76},{-84.8,76},{-84.8,69.8}}, color={0,
          0,127}));
  connect(T, west_flow.in_T) annotation (Line(points={{-110,30},{-62,30},{-62,-42},
          {-82.8,-42},{-82.8,-50.2}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Rectangular2D_manual;
