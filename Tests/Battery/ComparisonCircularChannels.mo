within DynTherM.Tests.Battery;
model ComparisonCircularChannels
  "This test compares the performance of circular channels which are modelled with different approaches"

// package Coolant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1);
package Coolant = Modelica.Media.Water.ConstantPropertyLiquidWater;


    parameter Temperature T_fluid=298.15;
    parameter MassFlowRate m_flow=0.04416;

    // Geometery
    parameter Length L = 1 "Lenght of the channel";
    parameter Length t = 0.012 "Thickness of the cold plate";
    parameter Length d = 0.014 "Center to center distance between the parallel channels";
    parameter Length R_int = 0.005 "Internal radius of the Channel";

    // Initialization
    parameter Temperature T_start_cell=298.15;
    parameter Temperature T_start_solid=298.15;
    parameter Temperature T_start_fluid=298.15;
    parameter Pressure P_start=3e5;
    parameter MassFlowRate m_flow_start=0.04416;
    parameter Velocity u_start=3;
    parameter Density rho_start=1e3;
    parameter Pressure dP_start=1e4;
    parameter ReynoldsNumber Re_start=5e3;
    parameter PrandtlNumber Pr_start=7;

    // Discretization
    parameter Integer N_cv= 6 "Number of control volumes in which the cooling channels are discretized";

    BoundaryConditions.pressure_sink          pressure_sink(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=150000,
    T_di=T_fluid,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{52,40},{72,60}})));
    BoundaryConditions.flow_source          flow_source(
    redeclare package Medium = Coolant,
    P_nom=400000,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{-94,62},{-70,38}})));
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
    annotation (Placement(transformation(extent={{-44,18},{28,82}})));
  Components.HeatTransfer.HeatCapacity heatCapacity(
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    T_start=313.15,
    C=1000000000000)
    "Imposing a constant surface temperature on the top of the channel by using a very high thermal inertia"
                     annotation (Placement(transformation(
        extent={{-14,-14},{14,14}},
        rotation=-90,
        origin={74,2})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier(Nx=N_cv, Ny=1)
                                                                  annotation (
      Placement(transformation(
        extent={{-12,-6},{12,6}},
        rotation=90,
        origin={32,10})));
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
    annotation (Placement(transformation(extent={{-44,-84},{28,-12}})));

  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier1(Nx=N_cv, Ny=1)
                                                                  annotation (
      Placement(transformation(
        extent={{-12,-6},{12,6}},
        rotation=90,
        origin={32,-14})));
    BoundaryConditions.flow_source          flow_source1(
    redeclare package Medium = Coolant,
    P_nom=400000,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{-92,-36},{-68,-60}})));
    BoundaryConditions.pressure_sink          pressure_sink1(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=150000,
    T_di=T_fluid,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{54,-58},{74,-38}})));
equation
  connect(flow_source.outlet, coldPlateCircularChannel1D.inlet)
    annotation (Line(points={{-70,50},{-44,50}}, color={0,0,0}));
  connect(coldPlateCircularChannel1D.outlet, pressure_sink.inlet)
    annotation (Line(points={{28,50},{52,50}}, color={0,0,0}));
  connect(heatFlowMultiplier.distributed, coldPlateCircularChannel1D.BottomSurface)
    annotation (Line(points={{28.4,10},{9.28,10},{9.28,34.96}},           color
        ={191,0,0}));
  connect(heatFlowMultiplier.single, heatCapacity.port)
    annotation (Line(points={{35.6,10},{54,10},{54,2},{60,2}},
                                             color={191,0,0}));
  connect(heatFlowMultiplier1.single, heatCapacity.port) annotation (Line(
        points={{35.6,-14},{54,-14},{54,2},{60,2}}, color={191,0,0}));
  connect(circularAsymmetricChannel1D.solid_surface_north, heatFlowMultiplier1.distributed)
    annotation (Line(points={{-35,-31.44},{-35,-14},{28.4,-14}}, color={191,0,0}));
  connect(flow_source1.outlet, circularAsymmetricChannel1D.inlet)
    annotation (Line(points={{-68,-48},{-44,-48}}, color={0,0,0}));
  connect(circularAsymmetricChannel1D.outlet, pressure_sink1.inlet)
    annotation (Line(points={{28,-48},{54,-48}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=1200, __Dymola_Algorithm="Dassl"));
end ComparisonCircularChannels;
