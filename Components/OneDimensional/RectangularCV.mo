within DynTherM.Components.OneDimensional;
model RectangularCV
  "Control volume modeling a portion of a rectangular channel"

  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  // Geometry
  input Real N=1 "Number of control volumes in parallel" annotation (Dialog(enable=true));
  parameter Length L "Length of the control volume" annotation (Dialog(tab="Geometry"));
  input Length W "Width of the control volume" annotation (Dialog(tab="Geometry", enable=true));
  input Length H "Height of the control volume" annotation (Dialog(tab="Geometry", enable=true));
  input Length t_north "Thickness of north wall" annotation (Dialog(tab="Geometry", enable=true));
  input Length t_east "Thickness of east wall" annotation (Dialog(tab="Geometry", enable=true));
  input Length t_south "Thickness of south wall" annotation (Dialog(tab="Geometry", enable=true));
  input Length t_west "Thickness of west wall" annotation (Dialog(tab="Geometry", enable=true));

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
  parameter MassFlowRate m_flow_start=1 "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start=20 "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
  parameter Density rho_start=1 "Density - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure dP_start=100 "Pressure drop - start value" annotation (Dialog(tab="Initialization"));
  parameter ReynoldsNumber Re_start=20e3 "Reynolds number - start value" annotation (Dialog(tab="Initialization"));
  parameter PrandtlNumber Pr_start=1.5 "Prandtl number - start value" annotation (Dialog(tab="Initialization"));

  Volume V_tot "Total volume";
  Volume V_fluid "Volume of fluid";
  Volume V_solid "Volume of solid walls";
  Mass m_tot "Total mass";
  Mass m_fluid "Mass of fluid";
  Mass m_solid "Mass of solid walls";
  HeatFlowRate Q "Heat flow rate - positive entering";

  DynTherM.CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
            {-94,6}},       rotation=0), iconTransformation(extent={{-90,-10},{-70,
            10}})));
  DynTherM.CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=
          -m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
            {106,6}},       rotation=0), iconTransformation(extent={{70,-10},{90,
            10}})));
  MassTransfer.RectangularPipe fluid(
    redeclare package Medium = Medium,
    allowFlowReversal=allowFlowReversal,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    m_flow_start=m_flow_start,
    P_start=P_start,
    T_start=T_start_fluid,
    X_start=X_start,
    u_start=u_start,
    rho_start(displayUnit="kg/m3") = rho_start,
    dP_start=dP_start,
    state_start=state_start,
    Re_start=Re_start,
    Pr_start=Pr_start,
    N=N,
    L=L,
    W=W,
    H=H)
    annotation (Placement(transformation(extent={{-40,-40},{40,40}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a solid_surface_north
    annotation (Placement(transformation(extent={{-88,80},{-72,96}}),
        iconTransformation(extent={{-70,60},{-50,80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a solid_surface_south
    annotation (Placement(transformation(extent={{22,80},{38,96}}),
        iconTransformation(extent={{10,60},{30,80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a solid_surface_west
    annotation (Placement(transformation(extent={{72,80},{88,96}}),
        iconTransformation(extent={{50,60},{70,80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a solid_surface_east
    annotation (Placement(transformation(extent={{-38,80},{-22,96}}),
        iconTransformation(extent={{-30,60},{-10,80}})));
  HeatTransfer.WallConduction solid_north(
    redeclare model Mat = Mat,
    N=N,
    t=t_north,
    A=W*L,
    Tstart=T_start_solid,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-100,66},{-60,36}})));
  HeatTransfer.WallConduction solid_east(
    redeclare model Mat = Mat,
    N=1,
    t=t_east,
    A=(H + 2*t_east)*L,
    Tstart=T_start_solid,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-50,66},{-10,36}})));
  HeatTransfer.WallConduction solid_south(
    redeclare model Mat = Mat,
    N=N,
    t=t_south,
    A=W*L,
    Tstart=T_start_solid,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{10,66},{50,36}})));
  HeatTransfer.WallConduction solid_west(
    redeclare model Mat = Mat,
    N=1,
    t=t_west,
    A=(H + 2*t_west)*L,
    Tstart=T_start_solid,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{60,66},{100,36}})));

equation
  V_tot = N*L*(H + t_north + t_south)*(W + t_east + t_west);
  V_fluid = N*L*H*W;
  V_tot = V_fluid + V_solid;
  m_tot = m_fluid + m_solid;
  m_fluid = fluid.rho*V_fluid;
  m_solid = Mat.rho*V_solid;
  Q = fluid.thermalPort.Q_flow;

  connect(inlet, fluid.inlet)
    annotation (Line(points={{-100,0},{-40,0}}, color={0,0,0}));
  connect(fluid.outlet, outlet)
    annotation (Line(points={{40,0},{100,0}}, color={0,0,0}));
  connect(solid_north.inlet, fluid.thermalPort) annotation (Line(points={{-80,45.9},
          {-80,30},{0,30},{0,15.2}}, color={191,0,0}));
  connect(solid_east.inlet, fluid.thermalPort) annotation (Line(points={{-30,45.9},
          {-30,30},{0,30},{0,15.2}}, color={191,0,0}));
  connect(solid_west.inlet, fluid.thermalPort) annotation (Line(points={{80,45.9},
          {80,30},{0,30},{0,15.2}}, color={191,0,0}));
  connect(solid_south.inlet, fluid.thermalPort) annotation (Line(points={{30,45.9},
          {30,30},{0,30},{0,15.2}}, color={191,0,0}));
  connect(solid_surface_north, solid_north.outlet)
    annotation (Line(points={{-80,88},{-80,56.1}}, color={191,0,0}));
  connect(solid_surface_east, solid_east.outlet)
    annotation (Line(points={{-30,88},{-30,56.1}}, color={191,0,0}));
  connect(solid_surface_south, solid_south.outlet)
    annotation (Line(points={{30,88},{30,56.1}}, color={191,0,0}));
  connect(solid_surface_west, solid_west.outlet)
    annotation (Line(points={{80,88},{80,56.1}}, color={191,0,0}));
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
          preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The model accounts for both mass transfer through the internal fluid control volume and heat transfer through the external solid control volume.</p>
</html>"));
end RectangularCV;
