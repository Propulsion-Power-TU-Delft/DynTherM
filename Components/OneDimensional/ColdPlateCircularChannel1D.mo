within DynTherM.Components.OneDimensional;
model ColdPlateCircularChannel1D
  "Circular channel in a cold plate with 1D spatial discretization"

  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  model CV = ColdPlateCircularChannelCV "Control volume";
  model I = DynTherM.Components.MassTransfer.PlenumSimple
    "Inertia between two adjacent control volumes";

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
 parameter Choices.PDropOpt DP_opt
    "Select the type of pressure drop to impose";
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  // Geometry
  parameter Length L "Channel length" annotation (Dialog(tab="Geometry"));
  parameter Length t "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
  parameter Length d "Center to center distance between the Channels"
                                                                     annotation (Dialog(tab="Geometry"));
  parameter Length R_int "Channel internal radius" annotation (Dialog(tab="Geometry"));
  parameter Length Roughness=0.015*10^(-3) "Channel roughness" annotation (Dialog(tab="Geometry"));
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

  CV cv[N_cv](
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    each N=N_channels,
    each L=L/N_cv,
    each t=t,
    each d=d,
    each R_int=R_int,
    each Roughness=Roughness,
    each T_start_solid=T_start_solid,
    each T_start_fluid=T_start_fluid,
    each P_start=P_start,
    each X_start=X_start,
    each state_start=state_start,
    each m_flow_start=m_flow_start,
    each u_start=u_start,
    each rho_start=rho_start,
    each dP_start=dP_start,
    each Re_start=Re_start,
    each Pr_start=Pr_start,
    each initOpt=initOpt,
    each DP_opt=DP_opt,
    each allowFlowReversal=allowFlowReversal);

  I inertia[N_cv-1](
    redeclare package Medium = Medium,
    each V=V_inertia,
    each P_start=P_start,
    each T_start=T_start_fluid,
    each X_start=X_start,
    each state_start=state_start,
    each m_flow_start=m_flow_start,
    each noInitialPressure=true,
    each noInitialTemperature=false,
    each initOpt=initOpt,
    each allowFlowReversal=allowFlowReversal);


  DynTherM.CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
            {-94,6}},       rotation=0), iconTransformation(extent={{-110,-10},{
            -90,10}})));
  DynTherM.CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=
          -m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
            {106,6}},       rotation=0), iconTransformation(extent={{90,-10},{110,
            10}})));

  CustomInterfaces.DistributedHeatPort_B TopSurface(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(extent={{8,14},{88,80}}), iconTransformation(
          extent={{8,14},{88,80}})));

  CustomInterfaces.DistributedHeatPort_B BottomSurface(Nx=N_cv, Ny=1)
    annotation (Placement(transformation(extent={{8,-80},{88,-14}}),
        iconTransformation(extent={{8,-80},{88,-14}})));
  CustomInterfaces.DistributedHeatPort_A EastSide(Nx=N_cv, Ny=Nt)
    annotation (
      Placement(transformation(extent={{-82,-82},{-12,-12}}),
        iconTransformation(extent={{-82,-82},{-12,-12}})));
  CustomInterfaces.DistributedHeatPort_A WestSide(Nx=N_cv, Ny=Nt)
    annotation (
      Placement(transformation(extent={{-82,12},{-12,82}}), iconTransformation(
          extent={{-84,12},{-14,82}})));
equation

  // thermal connections (Top,N and Bottom,S)
  for i in 1:N_cv loop
    connect(TopSurface.ports[i, 1], cv[i].NorthTop);
    connect(BottomSurface.ports[i, 1], cv[i].SouthBottom);
  end for;
  // thermal connections (Side ways, West side)
  for i in 1:N_cv loop
      connect(WestSide.ports[i,1], cv[i].NorthWestHor);
      connect(WestSide.ports[i,2], cv[i].West);
      connect(WestSide.ports[i,3], cv[i].SouthWestHor);
  end for;
  // thermal connections (Side ways, East side)
  for i in 1:N_cv loop
      connect(EastSide.ports[i,1], cv[i].NorthEastHor);
      connect(EastSide.ports[i,2], cv[i].East);
      connect(EastSide.ports[i,3], cv[i].SouthEastHor);
  end for;


  // internal flow connections
  for i in 1:(N_cv-1) loop
    connect(cv[i].outlet, inertia[i].inlet);
    connect(inertia[i].outlet, cv[i+1].inlet);
  end for;

  // boundary flow connections
  connect(inlet, cv[1].inlet);
  connect(outlet, cv[N_cv].outlet);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                      Rectangle(
          extent={{-100,40},{100,20}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
                      Rectangle(
          extent={{-100,-20},{100,-40}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Rectangle(extent={{-100,20},{100,-20}}, lineColor={0,0,0}),
        Line(
          points={{-60,20},{-60,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{-20,20},{-20,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{20,20},{20,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{60,20},{60,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash)}),         Diagram(coordinateSystem(
          preserveAspectRatio=false)),
    Documentation(info="<html>
<p><span style=\"font-family: Courier New;\">Model created by stacking CircularCV in series and adding SimplePlenum in between to improve solver robustness.</span></p>
</html>"));
end ColdPlateCircularChannel1D;
