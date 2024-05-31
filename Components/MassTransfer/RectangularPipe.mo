within DynTherM.Components.MassTransfer;
model RectangularPipe "Model of pipe with rectangular cross-section"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
  parameter Choices.PDropOpt DP_opt
    "Select the type of pressure drop to impose";

  parameter CustomUnits.HydraulicResistance Rh=1 "Hydraulic Resistance" annotation (Dialog(enable=DP_opt == Choices.PDropOpt.linear));
  parameter Pressure dP_fixed=0 "Fixed pressure drop" annotation (Dialog(enable=DP_opt == Choices.PDropOpt.fixed));

  // Initialization
  parameter MassFlowRate m_flow_start=1 "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325 "Pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start=300 "Temperature - start value" annotation (Dialog(tab="Initialization"));
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
  input Real N=1 "Number of pipes in parallel" annotation (Dialog(enable=true));
  parameter Length L "Length" annotation (Dialog(tab="Geometry"));
  input Length W "Width of rectangular channel" annotation (Dialog(tab="Geometry", enable=true));
  input Length H "Height of rectangular channel" annotation (Dialog(tab="Geometry", enable=true));

  model GEO =
    Components.MassTransfer.PipeGeometry.Rectangular (L=L, W=W, H=H) annotation (choicesAllMatching=true);

  // Heat transfer
  model HTC =
    Components.HeatTransfer.HTCorrelations.InternalConvection.Forrest (
       redeclare package Medium=Medium,
      Dh=geometry.Dh,
      phi_star=geometry.phi_star,
      Re=Re,
      Pr=Pr,
      state=state) annotation (choicesAllMatching=true);

  // Pressure drop
  model DPC =
    Components.MassTransfer.DPCorrelations.Forrest (
      redeclare package Medium=Medium,
      Dh=geometry.Dh,
      phi_star=geometry.phi_star,
      Re=Re) annotation (choicesAllMatching=true);

  HTC convection;
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
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=
          -m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{80,
            -20},{120,20}}, rotation=0), iconTransformation(extent={{90,-10},{
            110,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b thermalPort
    annotation (Placement(transformation(extent={{-10,28},{10,48}})));

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
  outlet.h_outflow = inStream(inlet.h_outflow) + thermalPort.Q_flow/inlet.m_flow;
  inlet.h_outflow = inStream(outlet.h_outflow) + thermalPort.Q_flow/inlet.m_flow;
  thermalPort.Q_flow = convection.ht*N*geometry.A_ht*(thermalPort.T - Medium.temperature(state));

  // Non-dimensional numbers
  rho = Medium.density(state);
  u = inlet.m_flow/(rho*N*geometry.A_cs);
  Re = rho*u*geometry.Dh/Medium.dynamicViscosity(state);
  Pr = Medium.specificHeatCapacityCp(state)*Medium.dynamicViscosity(state)/
    Medium.thermalConductivity(state);

  // Pressure drop
  if DP_opt == Choices.PDropOpt.fixed then
    dP = dP_fixed;
  elseif DP_opt == Choices.PDropOpt.correlation then
    dP = 0.5*friction.f*rho*u^2/geometry.Dh*geometry.L;
  elseif DP_opt == Choices.PDropOpt.linear then
    dP = Rh*inlet.m_flow;
  else
    dP = 0;
  end if;

  inlet.P - outlet.P = dP;

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
        Rectangle(extent={{-100,20},{100,-20}}, lineColor={0,0,0})}), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The convective heat transfer coefficient and the friction factor can be fixed by the user or computed with semi-empirical correlations.</p>
<p>The conductive heat transfer through the solid walls is not included in this model and must be treated separately.</p>
<p>The model can be used to reproduce the flow through many tubes in parallel. In that case, the mass flow rate is split equally among the different tubes.</p>
</html>"));
end RectangularPipe;
