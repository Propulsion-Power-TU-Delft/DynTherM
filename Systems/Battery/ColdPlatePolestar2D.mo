within DynTherM.Systems.Battery;
model ColdPlatePolestar2D
  "Model for a cold plate heat exchanger used in Polestar 2 battery Module, with 2D temperature distribution on the top and bottom surfaces"

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

   // For Results
  Pressure PressureDropColdplate "Total pressure drop in the cold plate";
  Volume V_plate "Total Volume of the plate";
  Volume V_fluid "Total Volume of the fluid";
  Mass m_fluid "Total mass of the fluid";
  Mass m_solid "Total mass of the solid";


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
    N_cv=N_cv)
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
    N_cv=N_cv)
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
    N_cv=N_cv)
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
    N_cv=N_cv)
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
    N_cv=N_cv)
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
    N_cv=N_cv)
    annotation (Placement(transformation(extent={{42,-58},{0,-92}})));
  CustomInterfaces.Adaptors.heatFlowInverter2D ChannelInvert1_6(Nx=N_cv, Ny=3)
    annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={-22,-62})));
  CustomInterfaces.TwoDimensional.HeatPort2D_A Bottom(Nx=N_cv, Ny=6)
    annotation (Placement(transformation(
        extent={{-32,-30},{32,30}},
        rotation=90,
        origin={140,0}), iconTransformation(extent={{-30,-62},{26,-6}})));
  CustomInterfaces.TwoDimensional.HeatPort2D_A Top(Nx=N_cv, Ny=6)
    annotation (
     Placement(transformation(
        extent={{-32,-30},{32,30}},
        rotation=90,
        origin={-140,0}), iconTransformation(extent={{-30,6},{26,62}})));
  CustomInterfaces.Adaptors.heatFlowInverter2D ChannelInvert1_2(Nx=N_cv, Ny=3)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={18,-30})));
  CustomInterfaces.Adaptors.heatFlowInverter2D ChannelInvert2_3(Nx=N_cv, Ny=3)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={-22,-8})));
  CustomInterfaces.Adaptors.heatFlowInverter2D ChannelInvert3_4(Nx=N_cv, Ny=3)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={20,24})));
  CustomInterfaces.Adaptors.heatFlowInverter2D ChannelInvert4_5(Nx=N_cv, Ny=3)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={-16,52})));
  CustomInterfaces.ZeroDimensional.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-88,
            -64},{-76,-52}}, rotation=0), iconTransformation(extent={{-96,2},{-84,
            14}})));
  CustomInterfaces.ZeroDimensional.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=
          -m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-34,
            -94},{-22,-82}}, rotation=0), iconTransformation(extent={{-96,-12},
            {-84,0}})));
  CustomInterfaces.Adaptors.heatFlowInverter1D Channel6ToTopInvert(Nx=N_cv) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={104,-14})));
  CustomInterfaces.Adaptors.heatFlowInverter1D Channel2ToTopInvert(Nx=N_cv) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={102,10})));
  CustomInterfaces.Adaptors.heatFlowInverter1D Channel4ToTopInvert(Nx=N_cv) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={102,30})));
  CustomInterfaces.Adaptors.heatFlowInverter1D Channel6ToBottomInvert(Nx=N_cv) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={-110,-16})));
  CustomInterfaces.Adaptors.heatFlowInverter1D Channel2ToBottomInvert(Nx=N_cv) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={-110,10})));
  CustomInterfaces.Adaptors.heatFlowInverter1D Channel4ToBottomInvert(Nx=N_cv) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={-110,30})));

equation
  PressureDropColdplate = Channel1.inlet.P - Channel6.outlet.P;
  V_plate = (6*d)*L*t;
  V_fluid = (pi*R_int*R_int) * L * 6;
  m_fluid = (Channel6.cv[end].circularPipe.rho +Channel1.cv[1].circularPipe.rho)*V_fluid/2;
  m_solid = Mat.rho*(V_plate-V_fluid);

  connect(Channel1.inlet, inlet) annotation (Line(points={{-44,-43},{-44,-44},{
          -64,-44},{-64,-58},{-82,-58}},
                                     color={0,0,0}));
  connect(ChannelInvert1_6.input2D, Channel1.EastSide) annotation (Line(points={
          {-26.14,-62.06},{-26.14,-62},{-32.34,-62},{-32.34,-51.93}}, color={28,
          108,200}));
  connect(ChannelInvert1_6.output2D, Channel6.EastSide) annotation (Line(points
        ={{-17.86,-62.06},{30.87,-62.06},{30.87,-67.01}}, color={28,108,200}));
  connect(Channel1.WestSide, ChannelInvert1_2.output2D) annotation (Line(points
        ={{-32.78,-34.07},{-32.78,-30.06},{13.86,-30.06}}, color={28,108,200}));
  connect(ChannelInvert1_2.input2D, Channel2.WestSide) annotation (Line(points={
          {22.14,-30.06},{32,-30.06},{32,-22.99},{31.29,-22.99}}, color={28,108,
          200}));
  connect(Channel2.EastSide, ChannelInvert2_3.input2D) annotation (Line(points={
          {30.87,-7.01},{30,-7.01},{30,-2},{-6,-2},{-6,-8.06},{-17.86,-8.06}},
        color={28,108,200}));
  connect(ChannelInvert2_3.output2D, Channel3.EastSide) annotation (Line(points
        ={{-26.14,-8.06},{-34,-8.06},{-34,7.54},{-30.87,7.54}}, color={28,108,200}));
  connect(Channel3.WestSide, ChannelInvert3_4.output2D) annotation (Line(points
        ={{-31.29,24.46},{-31.29,32},{6,32},{6,23.94},{15.86,23.94}}, color={28,
          108,200}));
  connect(ChannelInvert3_4.input2D, Channel4.WestSide) annotation (Line(points={
          {24.14,23.94},{32,23.94},{32,35.54},{32.78,35.54}}, color={28,108,200}));
  connect(Channel5.EastSide, ChannelInvert4_5.output2D) annotation (Line(points
        ={{-29.4,61.01},{-29.4,51.94},{-20.14,51.94}}, color={28,108,200}));
  connect(ChannelInvert4_5.input2D, Channel4.EastSide) annotation (Line(points={
          {-11.86,51.94},{-6,51.94},{-6,56},{32.34,56},{32.34,52.46}}, color={28,
          108,200}));
  connect(Channel6.outlet, outlet) annotation (Line(points={{0,-75},{0,-76},{
          -12,-76},{-12,-88},{-28,-88}},
                                     color={0,0,0}));
  connect(Channel6.TopSurface, Channel6ToTopInvert.input1D) annotation (Line(
        points={{10.92,-82.99},{10.92,-92},{92,-92},{92,-14.06},{99.86,-14.06}},
        color={0,140,72}));
  connect(Channel2.TopSurface, Channel2ToTopInvert.input1D) annotation (Line(
        points={{10.92,-22.99},{10.92,-40},{66,-40},{66,9.94},{97.86,9.94}},
        color={0,140,72}));
  connect(Channel4.TopSurface, Channel4ToTopInvert.input1D) annotation (Line(
        points={{11.44,35.54},{11.44,30},{97.86,30},{97.86,29.94}}, color={0,140,
          72}));
  connect(Channel6.BottomSurface, Channel6ToBottomInvert.output1D) annotation (
      Line(points={{10.92,-67.01},{10.92,-64},{-8,-64},{-8,-70},{-52,-70},{-52,-16.06},
          {-105.86,-16.06}}, color={0,140,72}));
  connect(Channel2.BottomSurface, Channel2ToBottomInvert.output1D) annotation (
      Line(points={{10.92,-7.01},{-4,-7.01},{-4,-18},{-46,-18},{-46,9.94},{-105.86,
          9.94}}, color={0,140,72}));
  connect(Channel4.BottomSurface, Channel4ToBottomInvert.output1D) annotation (
      Line(points={{11.44,52.46},{-4,52.46},{-4,40},{-98,40},{-98,29.94},{-105.86,
          29.94}}, color={0,140,72}));

   for i in 1:N_cv loop
     // Connections for Channel 6
    connect(Channel6ToBottomInvert.input1D.ports[i], Top.ports[i, 1]);
    connect(Channel6ToTopInvert.output1D.ports[i], Bottom.ports[i, 1]);

     // Connections for Channel 1
      connect(Channel1.BottomSurface.ports[i], Bottom.ports[i,2]);
      connect(Channel1.TopSurface.ports[i], Top.ports[i,2]);

     // Connections for Channel 2
    connect(Channel2ToBottomInvert.input1D.ports[i], Top.ports[i, 3]);
    connect(Channel2ToTopInvert.output1D.ports[i], Bottom.ports[i, 3]);

     // Connections for Channel 3
      connect(Channel3.BottomSurface.ports[i], Bottom.ports[i,4]);
      connect(Channel3.TopSurface.ports[i], Top.ports[i,4]);

     // Connections for Channel 4
    connect(Channel4ToBottomInvert.input1D.ports[i], Top.ports[i, 5]);
    connect(Channel4ToTopInvert.output1D.ports[i], Bottom.ports[i, 5]);

     // Connections for Channel 5
      connect(Channel5.BottomSurface.ports[i], Bottom.ports[i,6]);
      connect(Channel5.TopSurface.ports[i], Top.ports[i,6]);

   end for;

  // Sanity check
  assert(t >= 2 * R_int, "Thickness of the plate greater than channel diameter", AssertionLevel.warning);

  connect(Channel2.outlet, bend2_3.inlet)
    annotation (Line(points={{0,-15},{-70,-15},{-70,-8}}, color={0,0,0}));
  connect(Channel3.inlet, bend2_3.outlet)
    annotation (Line(points={{-42,16},{-70,16},{-70,12}}, color={0,0,0}));
  connect(bend4_5.inlet, Channel4.outlet)
    annotation (Line(points={{-68,46},{-68,44},{0,44}}, color={0,0,0}));
  connect(bend4_5.outlet, Channel5.inlet)
    annotation (Line(points={{-68,66},{-68,69},{-40,69}}, color={0,0,0}));
  connect(Channel1.outlet, bend1_2.inlet) annotation (Line(points={{0,-43},{0,-44},
          {54,-44},{54,-38}}, color={0,0,0}));
  connect(bend1_2.outlet, Channel2.inlet)
    annotation (Line(points={{54,-18},{54,-15},{42,-15}}, color={0,0,0}));
  connect(Channel3.outlet, bend3_4.inlet)
    annotation (Line(points={{0,16},{58,16},{58,20}}, color={0,0,0}));
  connect(bend3_4.outlet, Channel4.inlet)
    annotation (Line(points={{58,40},{58,44},{44,44}}, color={0,0,0}));
  connect(bend3_1.inlet, Channel5.outlet)
    annotation (Line(points={{78,-18},{78,69},{0,69}}, color={0,0,0}));
  connect(bend3_1.outlet, Channel6.inlet)
    annotation (Line(points={{78,-38},{78,-75},{42,-75}}, color={0,0,0}));
   annotation (Icon(coordinateSystem(extent={{-140,-100},{140,100}}),
                   graphics={Bitmap(extent={{-86,-62},{72,62}}, fileName=
              "modelica://DynTherM/Figures/Polestar_ColdPlateIcon.png")}),
      Diagram(coordinateSystem(extent={{-140,-100},{140,100}})),
    Documentation(info="<html>
<p>Using <i>ColdPlateCircularChannel1D</i> to model heat transfer through cold plate of the configuration shown in the figure below, while also considering the lateral heat transfer between the channels. </p>
<p><br>There is 2D temperature distribution on the top and bottom surfaces of the cooling plate.</p><p><br><img src=\"modelica://DynTherM/Figures/Polestar_ColdPlate.png\"/></p>
</html>"));
end ColdPlatePolestar2D;
