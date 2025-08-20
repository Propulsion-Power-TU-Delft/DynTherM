within DynTherM.Components.MassTransfer;
model CurvedBend "Pressure drop due to curved bend according to Modelica standard library"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";

  // Geometry
  parameter Length D "Diameter" annotation (Dialog(tab="Geometry"));
  parameter Length R_bend "Radius of curvature" annotation (Dialog(tab="Geometry"));
  parameter Angle delta "Bending angle (5° - 180°)" annotation (Dialog(tab="Geometry"));
  parameter Length ks=0.0015e-3 "Surface roughness" annotation (Dialog(tab="Geometry"));

  final parameter Modelica.Fluid.Dissipation.PressureLoss.Bend.dp_curvedOverall_IN_con In_con(
    d_hyd=D,
    delta=delta,
    K=ks,
    R_0=R_bend);

  // Initialization
  parameter MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325
    "Pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start=300
    "Temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start=20 "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
  parameter Density rho_start=1 "Density - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure dP_start=100 "Pressure drop - start value" annotation (Dialog(tab="Initialization",
        enable=option <> Choices.PDropOpt.fixed));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));

  CustomUnits.Inertance L "Inertance";
  Area A_cs "Cross-sectional area";
  Velocity u(start=u_start) "Flow velocity in the pipe";
  CustomUnits.MassFlux G "Mass flux";
  Density rho(start=rho_start) "Average density of fluid in the pipe";
  Pressure dP(start=dP_start) "Pressure drop";
  Medium.ThermodynamicState state "Average thermodynamic state";
  Medium.ThermodynamicState state_inlet "Inlet state";
  Medium.ThermodynamicState state_outlet "Outlet state";
  Modelica.Fluid.Dissipation.PressureLoss.Bend.dp_curvedOverall_IN_var In_var(eta=Medium.dynamicViscosity(state), rho=rho);

  CustomInterfaces.ZeroDimensional.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-120,
            -20},{-80,20}}, rotation=0), iconTransformation(extent={{-110,-10},
            {-90,10}})));
  CustomInterfaces.ZeroDimensional.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=
          -m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{80,
            -20},{120,20}}, rotation=0), iconTransformation(extent={{-10,90},{10,
            110}})));

equation
  state_inlet =
    Medium.setState_phX(inlet.P,
    actualStream(inlet.h_outflow),
    actualStream(inlet.Xi_outflow));
  state_outlet =
    Medium.setState_phX(outlet.P,
    actualStream(outlet.h_outflow),
    actualStream(outlet.Xi_outflow));
  state =
    Medium.setState_phX((inlet.P + outlet.P)/2,
    (actualStream(inlet.h_outflow) + actualStream(outlet.h_outflow))/2,
    inlet.Xi_outflow);

  A_cs = pi*D^2/4;
  L = D/2*pi/180*delta/A_cs;
  rho = Medium.density(state);

  // Mass balance
  inlet.m_flow + outlet.m_flow = 0;
  G = abs(inlet.m_flow)/A_cs;
  u = G/rho;

  if inlet.m_flow > 0 then
    inlet.P - outlet.P = dP;
  else
    outlet.P - inlet.P = dP;
  end if;

  // Energy balance
  inlet.h_outflow = inStream(outlet.h_outflow);
  outlet.h_outflow = inStream(inlet.h_outflow);

  // Independent composition mass balances
  inlet.Xi_outflow = inStream(outlet.Xi_outflow);
  outlet.Xi_outflow = inStream(inlet.Xi_outflow);

  dP = Modelica.Fluid.Dissipation.PressureLoss.Bend.dp_curvedOverall_DP(
    In_con,
    In_var,
    inlet.m_flow);

  // Sanity check
  assert(ks < D/2, "Parameter roughness of pipe ks must be less than radius of pipe d/2.");
  assert(5 <= delta*180/pi and delta*180/pi <= 180, "Parameter angle of pipe bend must between boundaries 5° < delta < 180°.");
  assert(0.49 <= R_bend/D and R_bend/D <= 15, "Realative curvatue R/d should be between boundaries 0.5 <= R/d <= 15. If ratio is greater deviations have to be accepted.", AssertionLevel.warning);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={                                                                                                                                                                  Line(origin={-60,60},    points={{-40,-20},
              {20,-20},{20,40}},                                                                                                                                                                                                        color={0,0,0},                          smooth = Smooth.Bezier), Line(origin={-20,20},    points={{-80,-60},
              {60,-60},{60,80}},                                                                                                                                                                                                        color={0,0,0},                          smooth = Smooth.Bezier),
        Line(
          points={{-40,100},{40,100}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Line(
          points={{-100,40},{-100,-40}},
          color={0,0,0},
          smooth=Smooth.Bezier),                                                                                                                                                                                                        Line(origin={-60,40},    points={{-40,-20},
              {40,-20},{40,60}},                                                                                                                                                                                                        color={0,0,0},                          smooth = Smooth.Bezier), Line(origin={-40,20},    points={{-60,-40},
              {60,-40},{60,80}},                                                                                                                                                                                                        color={0,0,0},                          smooth = Smooth.Bezier)}),
                                                                      Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Pressure drop model implemented in Modelica.Fluid.Dissipation.PressureLoss.Bend</p>
</html>"));
end CurvedBend;
