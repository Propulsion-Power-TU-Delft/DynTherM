within DynTherM.Tests.Battery;
model ColdPlatePolestar_EffectOfSidewaysHeatTransfer
  "This Test comparest the performance of a cold plate with a one in which there is no heat transfer between the channels"

 package Coolant = DynTherM.Media.IncompressibleTableBased.MEG_Polestar;
// package Coolant = Modelica.Media.Water.ConstantPropertyLiquidWater;

  // Cold Plate
  parameter Length L = 0.35 "Length of the channel" annotation (Dialog(tab="Geometry"));
  parameter Length t = 2*R_int + 0.002 "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
  parameter Length d = 0.01 "Center to center distance between the Channels" annotation (Dialog(tab="Geometry"));
  parameter Length R_int = 0.0025  "Channel internal radius" annotation (Dialog(tab="Geometry"));

  parameter Temperature T_fluid=298.15;
  parameter MassFlowRate m_flow=0.04416;

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
    parameter Integer N_cv = 2 "Number of control volumes in which the cooling channels are discretized";

  Systems.Battery.ColdPlatePolestar coldPlatePolestar(
    redeclare model Mat = Materials.AluminiumColdPlate,
    redeclare package Medium = Coolant,
    allowFlowReversal=true,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    L=L,
    t=t,
    d=d,
    R_int=R_int,
    T_start_solid(displayUnit="degC") = 298.15,
    T_start_fluid=298.15,
    m_flow_start=0.04416,
    rho_start(displayUnit="kg/m3") = 1e3,
    u_start=2,
    dP_start=200000000,
    Re_start=3e3,
    Pr_start=25,
    N_cv=N_cv,
    Nt=3,
    N_channels=1)
    annotation (Placement(transformation(extent={{-10,6},{166,132}})));
    BoundaryConditions.flow_source          flow_source1(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
      annotation (Placement(transformation(extent={{-14,90},{4,72}})));
    BoundaryConditions.pressure_sink          pressure_sink1(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false)
      annotation (Placement(transformation(extent={{-2,52},{-14,64}})));
  Components.HeatTransfer.HeatCapacity heatCapacity(
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    T_start=313.15,
    C=1000000000000)
    "Imposing a constant surface temperature on the top of the channel by using a very high thermal inertia"
                     annotation (Placement(transformation(
        extent={{-14,-14},{14,14}},
        rotation=-90,
        origin={108,-6})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplierColdPlate(Nx=N_cv,
      Ny=1) annotation (Placement(transformation(
        extent={{-13,4},{13,-4}},
        rotation=180,
        origin={83,24})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel5(
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
    N_channels=1) annotation (Placement(transformation(
        extent={{21.5,-22.5},{-21.5,22.5}},
        rotation=180,
        origin={-91.5,80.5})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel4(
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
    N_channels=1) annotation (Placement(transformation(
        extent={{-21.5,-22.5},{21.5,22.5}},
        rotation=180,
        origin={-61.5,46.5})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel3(
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
    N_channels=1) annotation (Placement(transformation(
        extent={{21.5,-22.5},{-21.5,22.5}},
        rotation=180,
        origin={-91.5,14.5})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel2(
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
    N_channels=1) annotation (Placement(transformation(
        extent={{-21.5,-22.5},{21.5,22.5}},
        rotation=180,
        origin={-65.5,-19.5})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel1(
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
    N_channels=1) annotation (Placement(transformation(
        extent={{21.5,-22.5},{-21.5,22.5}},
        rotation=180,
        origin={-91.5,-51.5})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel6(
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
    N_channels=1) annotation (Placement(transformation(
        extent={{-21.5,-22.5},{21.5,22.5}},
        rotation=180,
        origin={-59.5,-85.5})));
    BoundaryConditions.flow_source          flow_source2(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
      annotation (Placement(transformation(extent={{-146,-42},{-128,-60}})));
    BoundaryConditions.pressure_sink          pressure_sink2(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false)
      annotation (Placement(transformation(extent={{-98,-92},{-110,-80}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplierChannels(Nx=N_cv,
      Ny=1) annotation (Placement(transformation(
        extent={{-13,4},{13,-4}},
        rotation=270,
        origin={51,-10})));
equation
  connect(flow_source1.outlet,coldPlatePolestar. inlet) annotation (Line(points={{4,81},{
          4,80},{14,80},{14,74.04},{21.4286,74.04}},
        color={0,0,0}));
  connect(pressure_sink1.inlet,coldPlatePolestar. outlet) annotation (Line(
        points={{-2,58},{12,58},{12,65.22},{21.4286,65.22}},
        color={0,0,0}));
  connect(heatFlowMultiplierColdPlate.distributed, coldPlatePolestar.Bottom)
    annotation (Line(points={{83,26.4},{83,48},{76,48},{76,47.58},{76.7429,
          47.58}},
        color={191,0,0}));
  connect(heatFlowMultiplierColdPlate.single, heatCapacity.port)
    annotation (Line(points={{83,21.6},{83,-6},{94,-6}}, color={191,0,0}));
  connect(Channel1.outlet, Channel2.inlet) annotation (Line(points={{-70,-51.5},
          {-36,-51.5},{-36,-19.5},{-44,-19.5}}, color={0,0,0}));
  connect(Channel2.outlet, Channel3.inlet) annotation (Line(points={{-87,-19.5},
          {-126,-19.5},{-126,14.5},{-113,14.5}}, color={0,0,0}));
  connect(Channel3.outlet, Channel4.inlet) annotation (Line(points={{-70,14.5},{
          -62,14.5},{-62,12},{-28,12},{-28,44},{-40,44},{-40,46.5}}, color={0,0,
          0}));
  connect(Channel4.outlet, Channel5.inlet) annotation (Line(points={{-83,46.5},{
          -126,46.5},{-126,80.5},{-113,80.5}}, color={0,0,0}));
  connect(Channel5.outlet, Channel6.inlet) annotation (Line(points={{-70,80.5},{
          -20,80.5},{-20,-85.5},{-38,-85.5}}, color={0,0,0}));
  connect(flow_source2.outlet, Channel1.inlet) annotation (Line(points={{-128,-51},
          {-128,-51.5},{-113,-51.5}}, color={0,0,0}));
  connect(Channel6.outlet, pressure_sink2.inlet)
    annotation (Line(points={{-81,-85.5},{-82,-86},{-98,-86}}, color={0,0,0}));
  connect(Channel6.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-69.82,-96.075},{-69.82,-100},{26,-100},{26,-10},{
          48.6,-10}}, color={102,44,145}));
  connect(Channel2.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-75.82,-30.075},{-76,-30.075},{-76,-36},{26,-36},{
          26,-10},{48.6,-10}}, color={102,44,145}));
  connect(Channel3.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-81.18,3.925},{26,3.925},{26,-10},{48.6,-10}},
        color={102,44,145}));
  connect(Channel4.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-71.82,35.925},{-66,35.925},{-66,28},{26,28},{26,-10},
          {48.6,-10}}, color={102,44,145}));
  connect(Channel5.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-81.18,69.925},{-16,69.925},{-16,28},{26,28},{26,-10},
          {48.6,-10}}, color={102,44,145}));
  connect(Channel1.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-81.18,-62.075},{26,-62.075},{26,-10},{48.6,-10}},
        color={102,44,145}));
  connect(heatFlowMultiplierChannels.single, heatCapacity.port) annotation (
      Line(points={{53.4,-10},{84,-10},{84,-6},{94,-6}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},
            {140,100}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{140,100}})));
end ColdPlatePolestar_EffectOfSidewaysHeatTransfer;
