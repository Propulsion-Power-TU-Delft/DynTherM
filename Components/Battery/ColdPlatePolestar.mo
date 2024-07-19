within DynTherM.Components.Battery;
model ColdPlatePolestar
    "Model for a cold plate heat exchanger used in Polestar 2 battery Module"

  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material used for the plate" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater
                                                                                constrainedby
    Modelica.Media.Interfaces.PartialMedium  "Medium model" annotation(choicesAllMatching = true);

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

  OneDimensional.ColdPlateCircularChannel1D Channel1(
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

  OneDimensional.ColdPlateCircularChannel1D Channel2(
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
  OneDimensional.ColdPlateCircularChannel1D Channel3(
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
  OneDimensional.ColdPlateCircularChannel1D Channel4(
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
  OneDimensional.ColdPlateCircularChannel1D Channel5(
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
  OneDimensional.ColdPlateCircularChannel1D Channel6(
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
        origin={133,1}), iconTransformation(extent={{-32,-114},{56,-26}})));
  CustomInterfaces.DistributedHeatPort_A Top(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(
        extent={{-35,-35},{35,35}},
        rotation=90,
        origin={-133,1}), iconTransformation(extent={{-34,2},{54,90}})));
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
            {-76,-52}},     rotation=0), iconTransformation(extent={{-134,-4},{
            -122,8}})));
  CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=-
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-34,-94},
            {-22,-82}},     rotation=0), iconTransformation(extent={{-136,-28},
            {-124,-16}})));
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
    annotation (Line(points={{-29.4,61.01},{-29.4,51.94},{-20.14,51.94}}, color
        ={28,108,200}));
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
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},
            {140,100}}), graphics={Bitmap(
          extent={{-128,-110},{140,86}},
          imageSource="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",
          fileName="modelica://DynTherM/Figures/Polestar_ColdPlate.png")}),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{140,100}})));
end ColdPlatePolestar;
