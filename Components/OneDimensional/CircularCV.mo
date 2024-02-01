within DynTherM.Components.OneDimensional;
model CircularCV "Control volume modeling a portion of a circular channel"

  outer Components.Environment environment "Environmental properties";
  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Geometry
  parameter Integer N=1 "Number of control volumes in parallel";
  parameter Length L "Length of the control volume" annotation (Dialog(tab="Geometry"));
  parameter Length R_ext "External radius of the control volume" annotation (Dialog(tab="Geometry"));
  parameter Length R_int "Internal radius of the control volume" annotation (Dialog(tab="Geometry"));
  parameter Length Roughness=0.015*10^(-3) "Pipe roughness" annotation (Dialog(tab="Geometry"));

  // Initialization
  parameter Temperature T_start_solid=288.15
    "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start_fluid=288.15
    "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325
    "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start=
    Medium.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start=20 "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure dP_start=100 "Pressure drop - start value" annotation (Dialog(tab="Initialization"));
  parameter Choices.InitOpt initOpt=environment.initOpt
    "Initialization option" annotation (Dialog(tab="Initialization"));

  DynTherM.CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if environment.allowFlowReversal then
      -Modelica.Constants.inf else 0, start=m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
            {-94,6}},       rotation=0), iconTransformation(extent={{-90,-10},{-70,
            10}})));
  DynTherM.CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if environment.allowFlowReversal then
      +Modelica.Constants.inf else 0, start=-m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
            {106,6}},       rotation=0), iconTransformation(extent={{70,-10},{90,
            10}})));
  MassTransfer.CircularPipe fluid(
    redeclare package Medium = Medium,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    m_flow_start=m_flow_start,
    P_start=P_start,
    T_start=T_start_fluid,
    X_start=X_start,
    u_start=u_start,
    dP_start=dP_start,
    state_start=state_start,
    N=N,
    L=L,
    D=R_int*2,
    Roughness=Roughness)
    annotation (Placement(transformation(extent={{-40,-40},{40,40}})));
  HeatTransfer.TubeConduction solid(
    redeclare model Mat = Mat,
    N=N,
    coeff=1,
    L=L,
    R_ext=R_ext,
    R_int=R_int,
    Tstart=T_start_solid,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-28,70},{28,30}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a solid_surface
    annotation (Placement(transformation(extent={{-10,70},{10,90}}),
        iconTransformation(extent={{-10,70},{10,90}})));
equation
  connect(inlet, fluid.inlet)
    annotation (Line(points={{-100,0},{-40,0}}, color={0,0,0}));
  connect(fluid.outlet, outlet)
    annotation (Line(points={{40,0},{100,0}}, color={0,0,0}));
  connect(solid_surface, solid.outlet) annotation (Line(points={{0,80},{0,
          68.4},{3.55271e-15,68.4},{3.55271e-15,56.8}}, color={191,0,0}));
  connect(solid.inlet, fluid.thermalPort) annotation (Line(points={{
          3.55271e-15,43.2},{3.55271e-15,29.2},{0,29.2},{0,15.2}}, color={191,
          0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                      Rectangle(
          extent={{-80,40},{80,20}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
                      Rectangle(
          extent={{-80,-20},{80,-40}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Rectangle(extent={{-80,20},{80,-20}},   lineColor={0,0,0}),
        Rectangle(extent={{-100,60},{100,-60}}, lineColor={0,0,0},
          pattern=LinePattern.Dash)}), Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end CircularCV;
