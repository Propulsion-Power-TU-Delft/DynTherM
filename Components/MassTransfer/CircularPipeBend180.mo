within DynTherM.Components.MassTransfer;
model CircularPipeBend180
  "Pressure drop in a pipe elbow with circular cross-section"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";

  // Initialization
  parameter MassFlowRate m_flow_start=1 "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325 "Pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start=288.15 "Temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start=20 "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
  parameter Density rho_start=1 "Density - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure dP_start=100 "Pressure drop - start value" annotation (Dialog(tab="Initialization",
        enable=option <> Choices.PDropOpt.fixed));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter ReynoldsNumber Re_start=20e3 "Reynolds number - start value" annotation (Dialog(tab="Initialization"));
  parameter PrandtlNumber Pr_start=1.5 "Prandtl number - start value" annotation (Dialog(tab="Initialization"));

  // Geometry
  parameter Integer N=1 "Number of pipes in parallel";
  parameter Length L "Length" annotation (Dialog(tab="Geometry"));
  parameter Length D "Diameter" annotation (Dialog(tab="Geometry"));
  parameter Length R_bend "Radius of curvature of the elbow" annotation (Dialog(tab="Geometry"));
  parameter Length Roughness=0.015*10^(-3) "Roughness" annotation (Dialog(tab="Geometry"));
  parameter Angle theeta "Angle of bend" annotation(Dialog(enable = true));


  model GEO =
    Components.MassTransfer.PipeGeometry.Circular (L=L, D=D) annotation (choicesAllMatching=true);

  // Pressure drop
  model DPC =
    DynTherM.Components.MassTransfer.DPCorrelations.Idelchik (
      Dh=geometry.Dh,
      R_bend=R_bend,
      Re=Re,
      theeta=theeta) annotation (choicesAllMatching=true);
//

  DPC friction;
  GEO geometry;

  CustomUnits.MassFlux G "Mass flux";
  ReynoldsNumber Re(start=Re_start) "Reynolds number";
  PrandtlNumber Pr(start=Pr_start) "Prandtl number";
  Velocity u(start=u_start) "Flow velocity in the pipe";
  Density rho(start=rho_start) "Average density of fluid in the pipe";
  Pressure dP(start=dP_start) "Pressure drop";
  Medium.ThermodynamicState state "Average thermodynamic state";
  Medium.ThermodynamicState state_inlet "Inlet state";
  Medium.ThermodynamicState state_outlet "Outlet state";

  CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-120,
            -20},{-80,20}}, rotation=0), iconTransformation(extent={{-110,-10},
            {-90,10}})));
  CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=-
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{80,-20},
            {120,20}},      rotation=0), iconTransformation(extent={{90,-10},{
            110,10}})));

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

  // Mass balance
  inlet.m_flow + outlet.m_flow = 0;
  G = abs(inlet.m_flow)/(N*geometry.A_cs);

  // Independent composition mass balances
  inlet.Xi_outflow = inStream(outlet.Xi_outflow);
  outlet.Xi_outflow = inStream(inlet.Xi_outflow);

  // Energy balance
  outlet.h_outflow = inStream(inlet.h_outflow);
  inlet.h_outflow = inStream(outlet.h_outflow);

  // Non-dimensional numbers
  rho = Medium.density(state);
  u = inlet.m_flow/(rho*N*geometry.A_cs);
  Re = rho*u*geometry.Dh/Medium.dynamicViscosity(state);
  Pr = Medium.specificHeatCapacityCp(state)*Medium.dynamicViscosity(state)/
    Medium.thermalConductivity(state);

  // Pressure drop
  dP = 0.5*homotopy(friction.K_friction, 1.2)*rho*u^2;
  inlet.P - outlet.P = dP;

  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(
          points={{-100,20},{-40,60},{40,60},{100,20}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Line(
          points={{-100,-20},{-40,20},{40,20},{100,-20}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Line(
          points={{-100,-40},{-40,0},{40,0},{100,-40}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Line(
          points={{-100,40},{-40,80},{40,80},{100,40}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Line(
          points={{-100,40},{-100,-40}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Line(
          points={{100,40},{100,-40}},
          color={0,0,0},
          smooth=Smooth.Bezier)}),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Heat transfer is neglected.</p>
<h4>Reference:</h4>
<p>[1] N. Crawford et al. &quot;An Experimental Investigation into the Pressure Drop for Turbulent Flow 90 Degrees Elbow Bends&quot;, Institution of Mechanical Engineers. Proceedings. Part E: Journal of Process Mechanical Engineering, 2007.</p>
</html>"));
end CircularPipeBend180;
