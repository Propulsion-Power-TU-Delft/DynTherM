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
    parameter Pressure dP_start=8e3;
    parameter ReynoldsNumber Re_start=3e3;
    parameter PrandtlNumber Pr_start=30;

    // Discretization
    parameter Integer N_cv = 3 "Number of control volumes in which the cooling channels are discretized";

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
    dP_start=8000,
    Re_start=3e3,
    Pr_start=30,
    N_cv=N_cv,
    Nt=3,
    N_channels=1)
    annotation (Placement(transformation(extent={{22,20},{160,118}})));
  BoundaryConditions.ZeroDimensional.flow_source flow_source1(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-8,88},{8,72}})));
  BoundaryConditions.ZeroDimensional.pressure_sink pressure_sink1(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false)
    annotation (Placement(transformation(extent={{6,54},{-6,66}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier1D heatFlowMultiplierColdPlate(
    Nx=N_cv) annotation (Placement(transformation(
        extent={{-14,5},{14,-5}},
        rotation=180,
        origin={90,37})));
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
        origin={-91.5,84.5})));
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
        origin={-61.5,50.5})));
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
        origin={-91.5,18.5})));
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
        origin={-65.5,-15.5})));
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
        origin={-91.5,-47.5})));
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
        origin={-59.5,-81.5})));
  BoundaryConditions.ZeroDimensional.flow_source flow_source2(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-138,-38},{-120,-56}})));
  BoundaryConditions.ZeroDimensional.pressure_sink pressure_sink2(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false)
    annotation (Placement(transformation(extent={{-98,-88},{-110,-76}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier1D heatFlowMultiplierChannels(
    Nx=N_cv) annotation (Placement(transformation(
        extent={{-13,4},{13,-4}},
        rotation=270,
        origin={51,-20})));
  BoundaryConditions.ZeroDimensional.thermal FixedTemperature(
    T=313.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{108,-62},{128,50}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heatSensorChannels
    annotation (Placement(transformation(extent={{80,-30},{100,-10}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heatSensorColdPlate
    annotation (Placement(transformation(extent={{90,-2},{110,18}})));
equation
  connect(flow_source1.outlet,coldPlatePolestar. inlet) annotation (Line(points={{8,80},{
          20,80},{20,72.92},{46.6429,72.92}},
        color={0,0,0}));
  connect(pressure_sink1.inlet,coldPlatePolestar. outlet) annotation (Line(
        points={{6,60},{20,60},{20,66.06},{46.6429,66.06}},
        color={0,0,0}));
  connect(heatFlowMultiplierColdPlate.distributed, coldPlatePolestar.Bottom)
    annotation (Line(points={{90,40},{90.0143,40},{90.0143,52.34}},
        color={191,0,0}));
  connect(Channel1.outlet, Channel2.inlet) annotation (Line(points={{-70,-47.5},
          {-36,-47.5},{-36,-15.5},{-44,-15.5}}, color={0,0,0}));
  connect(Channel2.outlet, Channel3.inlet) annotation (Line(points={{-87,-15.5},
          {-126,-15.5},{-126,18.5},{-113,18.5}}, color={0,0,0}));
  connect(Channel3.outlet, Channel4.inlet) annotation (Line(points={{-70,18.5},{
          -62,18.5},{-62,16},{-28,16},{-28,48},{-40,48},{-40,50.5}}, color={0,0,
          0}));
  connect(Channel4.outlet, Channel5.inlet) annotation (Line(points={{-83,50.5},{
          -126,50.5},{-126,84.5},{-113,84.5}}, color={0,0,0}));
  connect(Channel5.outlet, Channel6.inlet) annotation (Line(points={{-70,84.5},{
          -20,84.5},{-20,-81.5},{-38,-81.5}}, color={0,0,0}));
  connect(flow_source2.outlet, Channel1.inlet) annotation (Line(points={{-120,-47},
          {-120,-47.5},{-113,-47.5}}, color={0,0,0}));
  connect(Channel6.outlet, pressure_sink2.inlet)
    annotation (Line(points={{-81,-81.5},{-82,-82},{-98,-82}}, color={0,0,0}));
  connect(Channel6.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-69.82,-92.075},{-69.82,-96},{26,-96},{26,-20},{48.6,
          -20}},      color={102,44,145}));
  connect(Channel2.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-75.82,-26.075},{-76,-26.075},{-76,-32},{26,-32},{
          26,-20},{48.6,-20}}, color={102,44,145}));
  connect(Channel3.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-81.18,7.925},{26,7.925},{26,-20},{48.6,-20}},
        color={102,44,145}));
  connect(Channel4.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-71.82,39.925},{-66,39.925},{-66,32},{26,32},{26,-20},
          {48.6,-20}}, color={102,44,145}));
  connect(Channel5.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-81.18,73.925},{-16,73.925},{-16,32},{26,32},{26,-20},
          {48.6,-20}}, color={102,44,145}));
  connect(Channel1.TopSurface, heatFlowMultiplierChannels.distributed)
    annotation (Line(points={{-81.18,-58.075},{26,-58.075},{26,-20},{48.6,-20}},
        color={102,44,145}));
  connect(heatFlowMultiplierChannels.single, heatSensorChannels.port_a)
    annotation (Line(points={{53.4,-20},{80,-20}}, color={191,0,0}));
  connect(heatSensorChannels.port_b, FixedTemperature.thermal) annotation (Line(
        points={{100,-20},{121.333,-20},{121.333,-6}},color={191,0,0}));
  connect(heatFlowMultiplierColdPlate.single, heatSensorColdPlate.port_a)
    annotation (Line(points={{90,34},{90,8}},          color={191,0,0}));
  connect(heatSensorColdPlate.port_b, FixedTemperature.thermal) annotation (
      Line(points={{110,8},{122,8},{122,-6},{121.333,-6}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},
            {140,100}}), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{140,100}})),
    Documentation(info="<html>
<p>The performance of the cold plate is affected as a result of heat transfer between the channels. This test quantitatively simulate this effect. </p>
<h4>Results</h4>
<p>For a surface temperature of 40<sup><span style=\"font-size: 6pt;\">0</span></sup>C, the cooling plate has a heat transfer rate of around 1241W at steady state conditions. Ideally, this would have been around 1294W if there there was no heat transfer between the channels. </p>
</html>"));
end ColdPlatePolestar_EffectOfSidewaysHeatTransfer;
