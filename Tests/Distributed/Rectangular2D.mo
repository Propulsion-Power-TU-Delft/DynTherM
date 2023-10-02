within DynTherM.Tests.Distributed;
model Rectangular2D
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.5)
    "Refrigerant";

  parameter Integer N_cv=3 "Number of longitudinal sections in which each channel is discretized";
  parameter Integer N_channels=10 "Number of channels";
  parameter Length L "Channel length";
  parameter Length W "Width of the control volume";
  parameter Length H "Height of the control volume";
  parameter Length t_ext "Thickness of external walls";
  parameter Length t_int "Thickness of internal walls";
  parameter MassFlowRate m_flow "Refrigerant mass flow rate";
  parameter Temperature T_in "Refrigerant inlet temperature";

  parameter HeatFlux phi;
  parameter Temperature T_start_solid
    "Temperature of solid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start_fluid
    "Temperature of fluid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325
    "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Refrigerant.nX]=Refrigerant.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Refrigerant.ThermodynamicState state_start=
    Refrigerant.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));

  inner Components.Environment environment(
    allowFlowReversal=false, initOpt=DynTherM.Choices.InitOpt.steadyState)
                                                  annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.TwoDimensional.RectangularChannels2D channel2D(
    redeclare package Medium = Refrigerant,
    L=L,
    H=H,
    W=W,
    t_int=t_int,
    t_ext=t_ext,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    N_cv=N_cv,
    N_channels=N_channels) annotation (Placement(transformation(extent={{-30,-30},{30,30}})));

  BoundaryConditions.thermal_flux_distributed thermal_distributed(
    N=N_cv,
    phi=phi*ones(N_cv))
    annotation (Placement(transformation(extent={{-30,50},{6,72}})));
  BoundaryConditions.pressure_sink_distributed pressure_sink_distributed(
      redeclare package Medium = Refrigerant, N=N_channels)
    annotation (Placement(transformation(extent={{66,-14},{94,14}})));
  BoundaryConditions.flow_source_distributed flow_source_distributed(redeclare
      package Medium = Refrigerant, N=N_channels,
    massFlow_di=m_flow/N_channels*ones(N_channels),
    T_di=T_in*ones(N_channels))
    annotation (Placement(transformation(extent={{-94,-14},{-66,14}})));
equation

  connect(channel2D.solid_surface_north, thermal_distributed.thermal_flux)
    annotation (Line(points={{-12,31.8},{-12,46.4},{-12,46.4},{-12,61}}, color={
          255,127,0}));
  connect(channel2D.outlet, pressure_sink_distributed.inlet) annotation (Line(
        points={{30,0},{48,0},{48,2.66454e-15},{66,2.66454e-15}},    color={0,0,
          0}));
  connect(flow_source_distributed.outlet, channel2D.inlet) annotation (Line(
        points={{-66.42,2.77556e-15},{-48.21,2.77556e-15},{-48.21,-2.22045e-15},
          {-30,-2.22045e-15}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Rectangular2D;
