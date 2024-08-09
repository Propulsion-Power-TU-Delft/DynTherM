within DynTherM.Systems.Battery;
model ColdPlatePolestar
  "Model for a cold plate heat exchanger used in Polestar 2 battery Module, with temperature varying only in the direction of fluid flow"

  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material used for the plate" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater
    constrainedby Modelica.Media.Interfaces.PartialMedium
                                                         "Medium model" annotation(choicesAllMatching = true);

     // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
 parameter Choices.PDropOpt DP_opt
    "Select the type of pressure drop to impose";
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  // Geometry
  parameter Length L "Length of the channel" annotation (Dialog(tab="Geometry"));
  parameter Length t "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
  parameter Length d "Center to center distance between the Channels" annotation (Dialog(tab="Geometry"));
  parameter Length R_int "Channel internal radius" annotation (Dialog(tab="Geometry"));
  parameter Volume V_inertia=1e-10 "Volume of the plenum placed between two consecutive control volumes" annotation (Dialog(tab="Geometry"));

   // Initialization
  parameter Temperature T_start_solid=288.15 "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start_fluid=288.15 "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325 "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Density rho_start=1 "Density - start value" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start=20 "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure dP_start=100 "Pressure drop - start value" annotation (Dialog(tab="Initialization"));
  parameter ReynoldsNumber Re_start=20e3 "Reynolds number - start value" annotation (Dialog(tab="Initialization"));
  parameter PrandtlNumber Pr_start=1.5 "Prandtl number - start value" annotation (Dialog(tab="Initialization"));

    // Discretization
  parameter Integer N_cv(min=1) "Number of control volumes in which the cooling channels are discretized";
  parameter Integer Nt=3  "Number of control volumes across the thickness of the cooling plate";
  parameter Integer N_channels(min=1) "Number of channels";

  Components.TwoDimensional.ColdPlateCircularChannel1D Channel1(
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    V_inertia=V_inertia,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    u_start=u_start,
    rho_start=rho_start,
    dP_start=dP_start,
    initOpt=initOpt,
    DP_opt=DP_opt,
    Re_start=Re_start,
    Pr_start=Pr_start,
    L=L,
    t=t,
    d=d,
    R_int=R_int,
    N_cv=N_cv,
    Nt=Nt,
    N_channels=N_channels)
    annotation (Placement(transformation(extent={{-44,-62},{0,-24}})));

  Components.TwoDimensional.ColdPlateCircularChannel1D Channel2(
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    DP_opt=DP_opt,
    V_inertia=V_inertia,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    u_start=u_start,
    rho_start=rho_start,
    dP_start=dP_start,
    initOpt=initOpt,
    Re_start=Re_start,
    Pr_start=Pr_start,
    L=L,
    t=t,
    d=d,
    R_int=R_int,
    N_cv=N_cv,
    Nt=Nt,
    N_channels=N_channels)
    annotation (Placement(transformation(extent={{42,2},{0,-32}})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel3(
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    DP_opt=DP_opt,
    V_inertia=V_inertia,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    u_start=u_start,
    rho_start=rho_start,
    dP_start=dP_start,
    initOpt=initOpt,
    Re_start=Re_start,
    Pr_start=Pr_start,
    L=L,
    t=t,
    d=d,
    R_int=R_int,
    N_cv=N_cv,
    Nt=Nt,
    N_channels=N_channels)
    annotation (Placement(transformation(extent={{-42,-2},{0,34}})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel4(
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    DP_opt=DP_opt,
    V_inertia=V_inertia,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    u_start=u_start,
    rho_start=rho_start,
    dP_start=dP_start,
    initOpt=initOpt,
    Re_start=Re_start,
    Pr_start=Pr_start,
    L=L,
    t=t,
    d=d,
    R_int=R_int,
    N_cv=N_cv,
    Nt=Nt,
    N_channels=N_channels)
    annotation (Placement(transformation(extent={{44,62},{0,26}})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel5(
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    DP_opt=DP_opt,
    V_inertia=V_inertia,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    u_start=u_start,
    rho_start=rho_start,
    dP_start=dP_start,
    initOpt=initOpt,
    Re_start=Re_start,
    Pr_start=Pr_start,
    L=L,
    t=t,
    d=d,
    R_int=R_int,
    N_cv=N_cv,
    Nt=Nt,
    N_channels=N_channels)
    annotation (Placement(transformation(extent={{-40,52},{0,86}})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel6(
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    DP_opt=DP_opt,
    V_inertia=V_inertia,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    u_start=u_start,
    rho_start=rho_start,
    dP_start=dP_start,
    initOpt=initOpt,
    Re_start=Re_start,
    Pr_start=Pr_start,
    L=L,
    t=t,
    d=d,
    R_int=R_int,
    N_cv=N_cv,
    Nt=Nt,
    N_channels=N_channels)
    annotation (Placement(transformation(extent={{42,-58},{0,-92}})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows ChannelInvert1_6(Nx=N_cv,
      Ny=3) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={-22,-62})));
  CustomInterfaces.DistributedHeatPort_A Bottom(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(
        extent={{-35,-35},{35,35}},
        rotation=90,
        origin={133,1}), iconTransformation(extent={{-30,-62},{26,-6}})));
  CustomInterfaces.DistributedHeatPort_A Top(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(
        extent={{-35,-35},{35,35}},
        rotation=90,
        origin={-133,1}), iconTransformation(extent={{-30,6},{26,62}})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows ChannelInvert1_2(Nx=N_cv,
      Ny=3) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={18,-30})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows ChannelInvert2_3(Nx=N_cv,
      Ny=3) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={-22,-8})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows ChannelInvert3_4(Nx=N_cv,
      Ny=3) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={20,24})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows ChannelInvert4_5(Nx=N_cv,
      Ny=3) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={-16,52})));
  CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-88,-64},
            {-76,-52}},     rotation=0), iconTransformation(extent={{-96,2},{-84,
            14}})));
  CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=-
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-34,-94},
            {-22,-82}},     rotation=0), iconTransformation(extent={{-96,-12},{-84,
            0}})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows Channel6ToTopInvert(Nx=N_cv,
      Ny=1) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={104,-14})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows Channel2ToTopInvert(Nx=N_cv,
      Ny=1) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={102,10})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows Channel4ToTopInvert(Nx=N_cv,
      Ny=1) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={102,30})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows Channel6ToBottomInvert(Nx=N_cv,
      Ny=1) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={-110,-16})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows Channel2ToBottomInvert(Nx=N_cv,
      Ny=1) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={-110,10})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows Channel4ToBottomInvert(Nx=N_cv,
      Ny=1) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={-110,30})));
equation
  connect(Channel2.inlet, Channel1.outlet) annotation (Line(points={{42,-15},{42,
          -14},{60,-14},{60,-43},{0,-43}}, color={0,0,0}));
  connect(Channel2.outlet, Channel3.inlet) annotation (Line(points={{0,-15},{0,-14},
          {-58,-14},{-58,16},{-42,16}},
                                      color={0,0,0}));
  connect(Channel3.outlet, Channel4.inlet)
    annotation (Line(points={{0,16},{54,16},{54,44},{44,44}},
                                                            color={0,0,0}));
  connect(Channel4.outlet, Channel5.inlet) annotation (Line(points={{0,44},{-54,
          44},{-54,70},{-40,70},{-40,69}}, color={0,0,0}));
  connect(Channel1.inlet, inlet) annotation (Line(points={{-44,-43},{-44,-44},{
          -64,-44},{-64,-58},{-82,-58}},
                                     color={0,0,0}));
  connect(ChannelInvert1_6.distributedHeatPort_in, Channel1.EastSide)
    annotation (Line(points={{-26.14,-62.06},{-26.14,-62},{-32.34,-62},{-32.34,-51.93}},
        color={28,108,200}));
  connect(ChannelInvert1_6.distributedHeatPort_out, Channel6.EastSide)
    annotation (Line(points={{-17.86,-62.06},{30.87,-62.06},{30.87,-67.01}},
        color={28,108,200}));
  connect(Channel1.WestSide, ChannelInvert1_2.distributedHeatPort_out)
    annotation (Line(points={{-32.78,-34.07},{-32.78,-30.06},{13.86,-30.06}},
        color={28,108,200}));
  connect(ChannelInvert1_2.distributedHeatPort_in, Channel2.WestSide)
    annotation (Line(points={{22.14,-30.06},{32,-30.06},{32,-22.99},{31.29,-22.99}},
        color={28,108,200}));
  connect(Channel2.EastSide, ChannelInvert2_3.distributedHeatPort_in)
    annotation (Line(points={{30.87,-7.01},{30,-7.01},{30,-2},{-6,-2},{-6,-8.06},
          {-17.86,-8.06}},color={28,108,200}));
  connect(ChannelInvert2_3.distributedHeatPort_out, Channel3.EastSide)
    annotation (Line(points={{-26.14,-8.06},{-34,-8.06},{-34,7.54},{-30.87,7.54}},
        color={28,108,200}));
  connect(Channel3.WestSide, ChannelInvert3_4.distributedHeatPort_out)
    annotation (Line(points={{-31.29,24.46},{-31.29,32},{6,32},{6,23.94},{15.86,
          23.94}}, color={28,108,200}));
  connect(ChannelInvert3_4.distributedHeatPort_in, Channel4.WestSide)
    annotation (Line(points={{24.14,23.94},{32,23.94},{32,35.54},{32.78,35.54}},
        color={28,108,200}));
  connect(Channel5.EastSide, ChannelInvert4_5.distributedHeatPort_out)
    annotation (Line(points={{-29.4,61.01},{-29.4,51.94},{-20.14,51.94}}, color=
         {28,108,200}));
  connect(ChannelInvert4_5.distributedHeatPort_in, Channel4.EastSide)
    annotation (Line(points={{-11.86,51.94},{-6,51.94},{-6,56},{32.34,56},{32.34,
          52.46}}, color={28,108,200}));

  connect(Channel6.outlet, outlet) annotation (Line(points={{0,-75},{0,-76},{
          -12,-76},{-12,-88},{-28,-88}},
                                     color={0,0,0}));
  connect(Channel6.inlet, Channel5.outlet) annotation (Line(points={{42,-75},{42,
          -76},{72,-76},{72,69},{0,69}}, color={0,0,0}));
  connect(Channel5.TopSurface, Top) annotation (Line(points={{-10.4,76.99},{-10.4,
          84},{-82,84},{-82,1},{-133,1}}, color={0,140,72}));
  connect(Channel3.TopSurface, Top) annotation (Line(points={{-10.92,24.46},{
          -10.92,28},{-82,28},{-82,1},{-133,1}},
                                          color={0,140,72}));
  connect(Channel1.TopSurface, Top) annotation (Line(points={{-11.44,-34.07},{-11.44,
          -26},{-82,-26},{-82,1},{-133,1}}, color={0,140,72}));
  connect(Channel1.BottomSurface, Bottom) annotation (Line(points={{-11.44,-51.93},
          {-11.44,-58},{86,-58},{86,1},{133,1}}, color={0,140,72}));
  connect(Channel3.BottomSurface, Bottom) annotation (Line(points={{-10.92,7.54},
          {-10.92,1},{133,1}}, color={0,140,72}));
  connect(Channel5.BottomSurface, Bottom) annotation (Line(points={{-10.4,61.01},
          {86,61.01},{86,1},{133,1}}, color={0,140,72}));
  connect(Channel6.TopSurface, Channel6ToTopInvert.distributedHeatPort_in)
    annotation (Line(points={{10.92,-82.99},{10.92,-92},{92,-92},{92,-14.06},{99.86,
          -14.06}}, color={0,140,72}));
  connect(Channel6ToTopInvert.distributedHeatPort_out, Bottom) annotation (Line(
        points={{108.14,-14.06},{116,-14.06},{116,1},{133,1}}, color={0,140,72}));
  connect(Channel2ToTopInvert.distributedHeatPort_out, Bottom) annotation (Line(
        points={{106.14,9.94},{116,9.94},{116,1},{133,1}}, color={0,140,72}));
  connect(Channel2.TopSurface, Channel2ToTopInvert.distributedHeatPort_in)
    annotation (Line(points={{10.92,-22.99},{10.92,-40},{66,-40},{66,9.94},{97.86,
          9.94}}, color={0,140,72}));
  connect(Channel4.TopSurface, Channel4ToTopInvert.distributedHeatPort_in)
    annotation (Line(points={{11.44,35.54},{11.44,30},{97.86,30},{97.86,29.94}},
        color={0,140,72}));
  connect(Channel4ToTopInvert.distributedHeatPort_out, Bottom) annotation (Line(
        points={{106.14,29.94},{106.14,30},{116,30},{116,1},{133,1}}, color={0,140,72}));

  connect(Channel6.BottomSurface, Channel6ToBottomInvert.distributedHeatPort_out)
    annotation (Line(points={{10.92,-67.01},{10.92,-64},{-8,-64},{-8,-70},{-52,
          -70},{-52,-16.06},{-105.86,-16.06}}, color={0,140,72}));
  connect(Channel6ToBottomInvert.distributedHeatPort_in, Top) annotation (Line(
        points={{-114.14,-16.06},{-120,-16.06},{-120,1},{-133,1}}, color={0,140,
          72}));
  connect(Channel2.BottomSurface, Channel2ToBottomInvert.distributedHeatPort_out)
    annotation (Line(points={{10.92,-7.01},{-4,-7.01},{-4,-18},{-46,-18},{-46,
          9.94},{-105.86,9.94}}, color={0,140,72}));
  connect(Channel4.BottomSurface, Channel4ToBottomInvert.distributedHeatPort_out)
    annotation (Line(points={{11.44,52.46},{-4,52.46},{-4,40},{-98,40},{-98,
          29.94},{-105.86,29.94}}, color={0,140,72}));
  connect(Channel4ToBottomInvert.distributedHeatPort_in, Top) annotation (Line(
        points={{-114.14,29.94},{-120,29.94},{-120,1},{-133,1}}, color={0,140,
          72}));
  connect(Channel2ToBottomInvert.distributedHeatPort_in, Top) annotation (Line(
        points={{-114.14,9.94},{-120,10},{-120,1},{-133,1}}, color={0,140,72}));

  // Sanity check
  assert(t >= 2 * R_int, "Thickness of the plate greater than channel diameter", AssertionLevel.warning);

  annotation (Icon(coordinateSystem(extent={{-140,-100},{140,100}}),
                   graphics={Bitmap(extent={{-86,-62},{72,62}}, fileName="modelica://DynTherM/Figures/Polestar_ColdPlate.png")}),
      Diagram(coordinateSystem(extent={{-140,-100},{140,100}})),
    Documentation(info="<html>
<p>Using ColdPlateCircularChannel1D to model heat transfer through cold plate of the configuration shown in the figure below, while also considering the lateral heat transfer between the channels. </p>
<p><br>There is 1D temperature distribution on the top and bottom surfaces of the cooling plate.</p>
<p><br><img src=\"modelica://DynTherM/Figures/Polestar_ColdPlate.png\"/></p>
</html>"));
end ColdPlatePolestar;
