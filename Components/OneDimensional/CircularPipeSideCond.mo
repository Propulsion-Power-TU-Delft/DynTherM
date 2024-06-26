within DynTherM.Components.OneDimensional;
model CircularPipeSideCond
"Circular channel implementing 1D spatial discretization"

  outer Components.Environment environment "Environmental properties";
  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  model CV = DynTherM.Components.MassTransfer.CircularPipeSideCond "Control volume";

  // Geometry
  parameter Modelica.Units.SI.Length L "Channel length" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length R_int "Channel internal radius" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length Roughness=0.015*10^(-3) "Channel roughness" annotation (Dialog(tab="Geometry"));

  // Initialization
  parameter Modelica.Units.SI.Temperature T_start_solid=288.15
    "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature T_start_fluid=288.15
    "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure P_start=101325
    "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.PDropOpt DP_opt
    "Select the type of pressure drop to impose";
  parameter Modelica.Units.SI.MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Choices.InitOpt initOpt=environment.initOpt
    "Initialization option" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N(min=1) "Number of longitudinal sections in which the tube is discretized";

  CV fluid_cv[N](
    redeclare package Medium = Medium,
    each L=L/N,
    each D=R_int*2,
    each Roughness=Roughness,
    each T_start=T_start_fluid,
    each P_start=P_start,
    each X_start=X_start,
    each state_start=state_start,
    each DP_opt=DP_opt,
    each m_flow_start=m_flow_start);

  DynTherM.CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if environment.allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
            {-94,6}},       rotation=0), iconTransformation(extent={{-110,-10},{
            -90,10}})));
  DynTherM.CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if environment.allowFlowReversal then +Modelica.Constants.inf else 0, start=
          -m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
            {106,6}},       rotation=0), iconTransformation(extent={{90,-10},{110,
            10}})));

  CustomInterfaces.DistributedHeatPort_B solid_surface(Nx=N, Ny=1)
                                                            annotation (
      Placement(transformation(extent={{26,18},{80,72}}),  iconTransformation(
          extent={{26,18},{80,72}})));

  CustomInterfaces.DistributedHeatPort_A ConductionPort_A(Nx=N, Ny=1)
                                                            annotation (Placement(
        transformation(extent={{-78,18},{-24,72}}), iconTransformation(extent={{
            -78,18},{-24,72}})));


  CustomInterfaces.DistributedHeatPort_A ConductionPort_B(Nx=N, Ny=1)
    annotation (Placement(transformation(extent={{-78,-72},{-24,-18}}),
        iconTransformation(extent={{-78,-72},{-24,-18}})));
equation
  // thermal connections
  for i in 1:N loop
    connect(solid_surface.ports[i,1], fluid_cv[i].thermalPort);
  end for;

  // Connections for Side Conduction
  for i in 1:N loop
    connect(ConductionPort_A.ports[i,1],fluid_cv[i].port_a);
    connect(ConductionPort_B.ports[i,1],fluid_cv[i].port_b);
  end for;

  // internal flow connections
  for i in 1:(N-1) loop
    connect(fluid_cv[i].outlet, fluid_cv[i+1].inlet);
  end for;

  // boundary flow connections
  connect(inlet, fluid_cv[1].inlet);
  connect(outlet, fluid_cv[N].outlet);

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
          preserveAspectRatio=false)));
end CircularPipeSideCond;
