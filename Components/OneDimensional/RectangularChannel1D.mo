within DynTherM.Components.OneDimensional;
model RectangularChannel1D
  "Rectangular channel implementing 1D spatial discretization"

  outer Components.Environment environment "Environmental properties";
  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  model CV = RectangularCV "Control volume";
  model I = DynTherM.Components.MassTransfer.PlenumSimple
    "Inertia between two adjacent control volumes";

  // Geometry
  parameter Length L "Channel length" annotation (Dialog(tab="Geometry"));
  input Length W "Width of the channel" annotation (Dialog(tab="Geometry", enable=true));
  input Length H "Height of the channel" annotation (Dialog(tab="Geometry", enable=true));
  input Length t_north "Thickness of north wall" annotation (Dialog(tab="Geometry", enable=true));
  input Length t_east "Thickness of east wall" annotation (Dialog(tab="Geometry", enable=true));
  input Length t_south "Thickness of south wall" annotation (Dialog(tab="Geometry", enable=true));
  input Length t_west "Thickness of west wall" annotation (Dialog(tab="Geometry", enable=true));
  parameter Volume V_inertia=1e-6 "Volume of the plenum placed between two consecutive control volumes" annotation (Dialog(tab="Geometry"));

  // Initialization
  parameter Temperature T_start_solid=288.15 "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start_fluid=288.15 "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325 "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start=
    Medium.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter MassFlowRate m_flow_start=1 "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start=20 "Flow velocity within one channel - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure dP_start=100 "Pressure drop - start value" annotation (Dialog(tab="Initialization"));
  parameter Choices.InitOpt initOpt=environment.initOpt "Initialization option" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N_cv(min=1) "Number of longitudinal sections in which the tube is discretized";
  input Real N_channels(min=1) "Number of channels in parallel" annotation (Dialog(enable=true));

  CV cv[N_cv](
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    each N=N_channels,
    each L=L/N_cv,
    each W=W,
    each H=H,
    each t_north=t_north,
    each t_east=t_east,
    each t_south=t_south,
    each t_west=t_west,
    each T_start_solid=T_start_solid,
    each T_start_fluid=T_start_fluid,
    each P_start=P_start,
    each X_start=X_start,
    each state_start=state_start,
    each u_start=u_start,
    each dP_start=dP_start,
    each initOpt=initOpt);

  I inertia[N_cv-1](
    redeclare package Medium = Medium,
    each V=V_inertia,
    each P_start=P_start,
    each T_start=T_start_fluid,
    each X_start=X_start,
    each state_start=state_start,
    each noInitialPressure=true,
    each noInitialTemperature=false,
    each initOpt=initOpt);

  Mass m_tot "Total mass";
  Mass m_fluid "Mass of fluid";
  Mass m_solid "Mass of solid walls";
  Pressure dP "Pressure drop";
  HeatFlowRate Q "Heat flow rate - positive entering";

  DynTherM.CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if environment.allowFlowReversal then -Modelica.Constants.inf else 0,
      start=m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
            {-94,6}},       rotation=0), iconTransformation(extent={{-110,-10},{
            -90,10}})));
  DynTherM.CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if environment.allowFlowReversal then +Modelica.Constants.inf else 0,
      start=-m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
            {106,6}},       rotation=0), iconTransformation(extent={{90,-10},{110,
            10}})));

  CustomInterfaces.DistributedHeatPort_B solid_surface_north(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(extent={{-90,16},{-60,76}}), iconTransformation(
          extent={{-90,16},{-60,76}})));
  CustomInterfaces.DistributedHeatPort_B solid_surface_east(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(extent={{-40,16},{-10,76}}),  iconTransformation(
          extent={{-40,16},{-10,76}})));
  CustomInterfaces.DistributedHeatPort_B solid_surface_south(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(extent={{10,16},{40,76}}),    iconTransformation(
          extent={{10,16},{40,76}})));
  CustomInterfaces.DistributedHeatPort_B solid_surface_west(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(extent={{60,16},{90,76}}),    iconTransformation(
          extent={{60,16},{90,76}})));

equation
  m_tot = sum(cv.m_tot);
  m_fluid = sum(cv.m_fluid);
  m_solid = sum(cv.m_solid);
  dP = sum(cv.fluid.dP);
  Q = sum(cv.Q);

  // thermal connections
  for i in 1:N_cv loop
    // north
    connect(solid_surface_north.ports[i,1], cv[i].solid_surface_north);

    // east
    connect(solid_surface_east.ports[i,1], cv[i].solid_surface_east);

    // south
    connect(solid_surface_south.ports[i,1], cv[i].solid_surface_south);

    // west
    connect(solid_surface_west.ports[i,1], cv[i].solid_surface_west);

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
</html>"));
end RectangularChannel1D;
