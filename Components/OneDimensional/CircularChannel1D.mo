within DynTherM.Components.OneDimensional;
model CircularChannel1D "Circular channel implementing 1D spatial discretization"

  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  model CV = CircularCV "Control volume";
  model I = DynTherM.Components.MassTransfer.PlenumSimple
    "Inertia between two adjacent control volumes";

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  // Geometry
  parameter Length L "Channel length" annotation (Dialog(tab="Geometry"));
  parameter Length R_ext "Channel external radius" annotation (Dialog(tab="Geometry"));
  parameter Length R_int "Channel internal radius" annotation (Dialog(tab="Geometry"));
  parameter Length Roughness=0.015*10^(-3) "Channel roughness" annotation (Dialog(tab="Geometry"));
  parameter Volume V_inertia=1e-6 "Volume of the plenum placed between two consecutive control volumes" annotation (Dialog(tab="Geometry"));

  // Initialization
  parameter Temperature T_start_solid=288.15 "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start_fluid=288.15 "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325 "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter MassFlowRate m_flow_start=1 "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N_cv(min=1) "Number of control volumes in which the cooling channels are discretized";
  parameter Integer N_channels(min=1) "Number of channels in parallel";

  CV cv[N_cv](
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    each N=N_channels,
    each L=L/N_cv,
    each R_ext=R_ext,
    each R_int=R_int,
    each Roughness=Roughness,
    each T_start_solid=T_start_solid,
    each T_start_fluid=T_start_fluid,
    each P_start=P_start,
    each X_start=X_start,
    each state_start=state_start,
    each m_flow_start=m_flow_start,
    each initOpt=initOpt,
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

  Mass m_tot "Total mass";
  Mass m_fluid "Mass of fluid";
  Mass m_solid "Mass of solid walls";
  Pressure dP "Pressure drop";
  HeatFlowRate Q "Heat flow rate - positive entering";

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

  CustomInterfaces.DistributedHeatPort_B solid_surface(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(extent={{-40,14},{40,80}}), iconTransformation(
          extent={{-40,14},{40,80}})));

equation
  m_tot = sum(cv.m_tot);
  m_fluid = sum(cv.m_fluid);
  m_solid = sum(cv.m_solid);
  dP = sum(cv.fluid.dP);
  Q = sum(cv.Q);

  // thermal connections
  for i in 1:N_cv loop
    connect(solid_surface.ports[i,1], cv[i].solid_surface);
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
end CircularChannel1D;
