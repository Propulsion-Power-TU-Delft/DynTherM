within DynTherM.Tests.Battery;
model ComparisonCircularChannels
  "This test compares the performance of circular channels which are modelled with different approaches"

 package Coolant = DynTherM.Media.IncompressibleTableBased.MEG_Polestar;
// package Coolant = Modelica.Media.Water.StandardWater;
// package Coolant = Modelica.Media.Water.ConstantPropertyLiquidWater;

    parameter Temperature T_fluid=298.15;
    parameter MassFlowRate m_flow=0.04416;

    // Geometery
    parameter Length L = 0.35 "Lenght of the channel";
    parameter Length t = 0.006 "Thickness of the cold plate";
    parameter Length d = 0.007 "Center to center distance between the parallel channels";
    parameter Length R_int = 0.0025 "Internal radius of the Channel";

    // Initialization
    parameter Temperature T_start_solid=298.15;
    parameter Temperature T_start_fluid=298.15;
    parameter Pressure P_start=1e5;
    parameter MassFlowRate m_flow_start=0.04416;
    parameter Velocity u_start=2;
    parameter Density rho_start=1e3;
    parameter Pressure dP_start=2e3;
    parameter ReynoldsNumber Re_start=3e3;
    parameter PrandtlNumber Pr_start=30;

    // Discretization
    parameter Integer N_cv= 2 "Number of control volumes in which the cooling channels are discretized";

    BoundaryConditions.pressure_sink          pressure_sink(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=100000,
    T_di=T_fluid,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{56,32},{76,52}})));
    BoundaryConditions.flow_source          flow_source(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{-94,54},{-70,30}})));
  Components.TwoDimensional.ColdPlateCircularChannel1D
    coldPlateCircularChannel1D(
    redeclare package Medium = Coolant,
    allowFlowReversal=true,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    L=L,
    t(displayUnit="mm") = t,
    d(displayUnit="mm") = d,
    R_int(displayUnit="mm") = R_int,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    m_flow_start=m_flow_start,
    rho_start=rho_start,
    u_start=u_start,
    dP_start=dP_start,
    Re_start=Re_start,
    Pr_start=Pr_start,
    N_cv=N_cv,
    Nt=3,
    N_channels=1)
    annotation (Placement(transformation(extent={{-38,10},{34,74}})));
  Components.HeatTransfer.HeatCapacity heatCapacity(
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    T_start=313.15,
    C=1000000000000)
    "Imposing a constant surface temperature on the top of the channel by using a very high thermal inertia"
                     annotation (Placement(transformation(
        extent={{-14,-14},{14,14}},
        rotation=-90,
        origin={80,0})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier(Nx=N_cv, Ny=1)
                                                                  annotation (
      Placement(transformation(
        extent={{-12,-6},{12,6}},
        rotation=90,
        origin={40,10})));

  Components.OneDimensional.CircularAsymmetricChannel1D
    circularAsymmetricChannel1D(
    redeclare package Medium = Coolant,
    allowFlowReversal=true,
    L=L,
    R_ext_north(displayUnit="mm") = t/2,
    R_ext_east(displayUnit="mm") = d/2,
    R_ext_south(displayUnit="mm") = t/2,
    R_ext_west(displayUnit="mm") = d/2,
    R_int(displayUnit="mm") = R_int,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    m_flow_start=m_flow_start,
    rho_start=rho_start,
    u_start=u_start,
    dP_start=dP_start,
    Re_start=Re_start,
    Pr_start=Pr_start,
    N_cv=N_cv,
    N_channels=1)
    annotation (Placement(transformation(extent={{-40,-82},{32,-10}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier1(Nx=N_cv, Ny=1)
                                                                  annotation (
      Placement(transformation(
        extent={{-12,-6},{12,6}},
        rotation=90,
        origin={40,-10})));
    BoundaryConditions.flow_source          flow_source1(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{-96,-34},{-72,-58}})));
    BoundaryConditions.pressure_sink          pressure_sink1(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=100000,
    T_di=T_fluid,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{56,-56},{76,-36}})));
equation
  connect(flow_source.outlet, coldPlateCircularChannel1D.inlet)
    annotation (Line(points={{-70,42},{-38,42}}, color={0,0,0}));
  connect(heatFlowMultiplier.distributed, coldPlateCircularChannel1D.BottomSurface)
    annotation (Line(points={{36.4,10},{16,10},{16,18},{15.28,18},{15.28,26.96}},
                                                                          color
        ={191,0,0}));
  connect(heatFlowMultiplier.single, heatCapacity.port)
    annotation (Line(points={{43.6,10},{60,10},{60,0},{66,0}},
                                             color={191,0,0}));
  connect(coldPlateCircularChannel1D.outlet, pressure_sink.inlet)
    annotation (Line(points={{34,42},{56,42}}, color={0,0,0}));
  connect(heatFlowMultiplier1.single, heatCapacity.port) annotation (Line(
        points={{43.6,-10},{60,-10},{60,0},{66,0}},     color={191,0,0}));
  connect(circularAsymmetricChannel1D.solid_surface_north, heatFlowMultiplier1.distributed)
    annotation (Line(points={{-31,-29.44},{-30.5,-29.44},{-30.5,-10},{36.4,-10}},
        color={191,0,0}));
  connect(flow_source1.outlet, circularAsymmetricChannel1D.inlet)
    annotation (Line(points={{-72,-46},{-40,-46}}, color={0,0,0}));
  connect(circularAsymmetricChannel1D.outlet, pressure_sink1.inlet)
    annotation (Line(points={{32,-46},{56,-46}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=1200, __Dymola_Algorithm="Dassl"));
end ComparisonCircularChannels;
