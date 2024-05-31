within DynTherM.Components.MassTransfer;
model ActuatorDisk "Simple actuator disk model"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  outer Components.Environment environment "Environmental properties";

  // Options
  parameter Boolean set_m_flow=false "If true, mass flow rate through the actuator disk is prescribed by the user";
  parameter Boolean set_T=false "If true, thrust provided/absorbed by the actuator disk is prescribed by the user";
  parameter Boolean set_W=false "If true, power provided/absorbed by the actuator disk is prescribed by the user";
  parameter MassFlowRate m_flow_fixed=0 "Mass flow rate through the actuator disk - value prescribed by the user" annotation(Dialog(enable=set_m_flow));
  parameter Force T_fixed=0 "Thrust provided/absorbed by the actuator disk - value prescribed by the user" annotation(Dialog(enable=set_T));
  parameter Power W_fixed=0 "Power provided/absorbed by the actuator disk - value prescribed by the user" annotation(Dialog(enable=set_W));

  // Initialization
  parameter Medium.AbsolutePressure P_start=101325 "Pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.Temperature T_start=300 "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.MassFraction X_start[Medium.nX]=Medium.reference_X "Start gas composition" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));

  parameter Length R_disk "Radius of the actuator disk";

  Area A_disk "Area of the actuator disk";
  Velocity V_disk "Flow velocity at the actuator disk section";
  Velocity V_out "Flow velocity at the outlet section";
  Force T "Thrust provided/absorbed by the actuator disk";
  Power W "Power provided/absorbed by the actuator disk";
  Pressure P_up "Pressure upstream of the actuator disk";
  Pressure P_down "Pressure downstream of the actuator disk";
  Density rho_amb "Ambient density";
  SpecificEnthalpy h_amb "Ambient enthalpy";

  CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{80,-20},
            {120,20}},      rotation=0), iconTransformation(extent={{90,-10},{
            110,10}})));

equation
  h_amb = Medium.specificEnthalpy(environment.state_amb);
  rho_amb = Medium.density(environment.state_amb);

  if set_m_flow then
    outlet.m_flow + m_flow_fixed = 0;
  end if;

  if set_T then
    T = T_fixed;
  end if;

  if set_W then
    W = W_fixed;
  end if;

  // Mass balance
  outlet.m_flow + rho_amb*V_disk*A_disk = 0;

  // Independent composition mass balances
  environment.X_amb = outlet.Xi_outflow;

  // Energy balance
  outlet.h_outflow + V_out^2/2 = h_amb + environment.V_inf^2/2 + W/(rho_amb*V_disk*A_disk);

  // Bernoulli equation
  environment.P_amb + rho_amb*environment.V_inf^2/2 = P_up + rho_amb*V_disk^2/2;
  outlet.P + rho_amb*V_out^2/2 = P_down + rho_amb*V_disk^2/2;

  // Perfromance metrics
  A_disk = pi*R_disk^2;
  V_disk = (environment.V_inf + V_out)/2;
  T = A_disk*(P_down - P_up);
  W = T*V_disk;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(
          points={{-100,80},{-30,74},{30,36},{100,30}},
          color={0,0,0},
          smooth=Smooth.Bezier,
          thickness=0.5),
        Line(
          points={{-100,-80},{-30,-74},{30,-36},{100,-30}},
          color={0,0,0},
          smooth=Smooth.Bezier,
          thickness=0.5),
        Rectangle(
          extent={{-8,50},{8,-50}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Line(points={{-100,60},{-70,60}}, color={0,0,0}),
        Line(points={{-80,52},{-70,60}}, color={0,0,0}),
        Line(points={{-70,60},{-80,68}}, color={0,0,0}),
        Line(points={{-100,40},{-70,40}}, color={0,0,0}),
        Line(points={{-80,32},{-70,40}}, color={0,0,0}),
        Line(points={{-70,40},{-80,48}}, color={0,0,0}),
        Line(points={{-100,20},{-70,20}}, color={0,0,0}),
        Line(points={{-80,12},{-70,20}}, color={0,0,0}),
        Line(points={{-70,20},{-80,28}}, color={0,0,0}),
        Line(points={{-100,-20},{-70,-20}}, color={0,0,0}),
        Line(points={{-80,-28},{-70,-20}}, color={0,0,0}),
        Line(points={{-70,-20},{-80,-12}}, color={0,0,0}),
        Line(points={{-100,-40},{-70,-40}}, color={0,0,0}),
        Line(points={{-80,-48},{-70,-40}}, color={0,0,0}),
        Line(points={{-70,-40},{-80,-32}}, color={0,0,0}),
        Line(points={{-100,-60},{-70,-60}}, color={0,0,0}),
        Line(points={{-80,-68},{-70,-60}}, color={0,0,0}),
        Line(points={{-70,-60},{-80,-52}}, color={0,0,0}),
        Line(points={{60,20},{100,20}}, color={0,0,0}),
        Line(points={{90,12},{100,20}}, color={0,0,0}),
        Line(points={{100,20},{90,28}}, color={0,0,0}),
        Line(points={{60,-20},{100,-20}}, color={0,0,0}),
        Line(points={{90,-28},{100,-20}}, color={0,0,0}),
        Line(points={{100,-20},{90,-12}}, color={0,0,0})}),      Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><b>Assumptions:</b></p>
<p>1. No flow rotation</p>
<p>2. Incompressible flow</p>
<p>3. Thrust uniformly distributed over the actuator disk</p>
<p>4. Static pressure upstream and downstream the control volume matches the ambient value</p>
<p><br><b>Reference:</b></p>
<p>[1] P. M. Sforza. &quot;Theory of Aerospace Propulsion, Chapter 10: Propellers&quot;, 2017</p>
</html>"));
end ActuatorDisk;
