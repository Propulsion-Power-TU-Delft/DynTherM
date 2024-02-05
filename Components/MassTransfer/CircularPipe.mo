within DynTherM.Components.MassTransfer;
model CircularPipe "Model of pipe with circular cross-section"

  outer DynTherM.Components.Environment environment "Environmental properties";
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Options
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  parameter DynTherM.Choices.PDropOpt DP_opt
    "Select the type of pressure drop to impose";
  parameter DynTherM.CustomUnits.HydraulicResistance Rh=1 "Hydraulic Resistance" annotation (Dialog(enable=DP_opt == Choices.PDropOpt.linear));

  // Initialization
  parameter MassFlowRate m_flow_start=1 "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start[2]={101325,101325} "Pressure - start value (inlet - outlet)" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start[2]={288.15,288.15} "Temperature - start value (inlet - outlet)" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start[N_cv,1]=20*ones(N_cv,1) "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
  parameter Density rho_start[N_cv,1]=ones(N_cv,1) "Density - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure dP_start[N_cv,1]=100*ones(N_cv,1) "Pressure drop - start value" annotation (Dialog(tab="Initialization",
        enable=option <> Choices.PDropOpt.fixed));
  parameter Pressure dP_fixed=0 "Fixed pressure drop" annotation (Dialog(enable=DP_opt == Choices.PDropOpt.fixed));
  parameter Medium.ThermodynamicState state_inlet_start=
    Medium.setState_pTX(P_start[1], T_start[1], X_start)
    "Starting inlet thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_outlet_start=
    Medium.setState_pTX(P_start[2], T_start[2], X_start)
    "Starting outlet thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter ReynoldsNumber Re_start[N_cv,1]=20e3*ones(N_cv,1)
    "Reynolds number - start value" annotation (Dialog(tab="Initialization"));
  parameter PrandtlNumber Pr_start[N_cv,1]=1.5*ones(N_cv,1)
    "Prandtl number - start value" annotation (Dialog(tab="Initialization"));

  // Geometry
  parameter Integer N_cv=1 "Number of control volumes";
  parameter Integer N_pipes=1 "Number of pipes in parallel";
  parameter Length L "Length" annotation (Dialog(tab="Geometry"));
  parameter Length D "Diameter" annotation (Dialog(tab="Geometry"));
  parameter Length Roughness=0.015*10^(-3) "Roughness" annotation (Dialog(tab="Geometry"));

  model GEO =
    DynTherM.Components.MassTransfer.PipeGeometry.Circular (L=L, D=D) annotation (choicesAllMatching=true);

  // Heat transfer
  model HTC =
    DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.DittusBoelter
      (redeclare package Medium=Medium,
      Nx=N_cv,
      Ny=1,
      Dh=geometry.Dh,
      T_in=Medium.temperature(state_inlet),
      T_out=Medium.temperature(state_outlet),
      Re=Re,
      Pr=Pr,
      state=state) annotation (choicesAllMatching=true);

  // Pressure drop
  model DPC =
    DynTherM.Components.MassTransfer.DPCorrelations.DarcyWeisbach (
      redeclare package Medium=Medium,
      Nx=N_cv,
      Ny=1,
      Dh=geometry.Dh,
      Re=Re,
      Roughness=Roughness) annotation (choicesAllMatching=true);

  HTC convection;
  DPC friction;
  GEO geometry;

  DynTherM.CustomUnits.MassFlux G "Mass flux";
  ReynoldsNumber Re[N_cv,1](start=Re_start) "Reynolds number";
  PrandtlNumber Pr[N_cv,1](start=Pr_start) "Prandtl number";
  Velocity u[N_cv,1](start=u_start) "Flow velocity in the pipe";
  Density rho[N_cv,1](start=rho_start) "Average density of fluid in the pipe";
  Pressure dP_cv[N_cv,1](start=dP_start) "Pressure drop in each control volume";
  Pressure dP(start=sum(dP_start)) "Pressure drop";
  Medium.ThermodynamicState state[N_cv,1] "Average thermodynamic state";
  Medium.ThermodynamicState state_inlet "Inlet state";
  Medium.ThermodynamicState state_outlet "Outlet state";

  DynTherM.CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start[1]),
    h_outflow(start=Medium.specificEnthalpy(state_inlet_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-120,
            -20},{-80,20}}, rotation=0), iconTransformation(extent={{-110,-10},
            {-90,10}})));
  DynTherM.CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=
          -m_flow_start),
    P(start=P_start[2]),
    h_outflow(start=Medium.specificEnthalpy(state_outlet_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{80,
            -20},{120,20}}, rotation=0), iconTransformation(extent={{90,-10},{
            110,10}})));
  DynTherM.CustomInterfaces.DistributedHeatPort_B thermalPort(Nx=N_cv, Ny=1) annotation (Placement(transformation(extent={{-40,8},{40,88}}),
        iconTransformation(extent={{-40,8},{40,88}})));

equation
  state_inlet = Medium.setState_phX(
    inlet.P,
    actualStream(inlet.h_outflow),
    actualStream(inlet.Xi_outflow));
  state_outlet = Medium.setState_phX(
    outlet.P,
    actualStream(outlet.h_outflow),
    actualStream(outlet.Xi_outflow));

  for i in 1:N_cv loop
    // Average thermodynamic state of the control volume
    if i == 1 then
      state[i,1] = Medium.setState_phX(inlet.P - dP_cv[i,1]/2,
        actualStream(inlet.h_outflow) + thermalPort.ports[i,1].Q_flow/(2*inlet.m_flow),
        inlet.Xi_outflow);
    else
      state[i,1] = Medium.setState_phX(Medium.pressure(state[i-1,1]) - dP_cv[i,1],
        Medium.specificEnthalpy(state[i-1,1]) + thermalPort.ports[i,1].Q_flow/inlet.m_flow,
        inlet.Xi_outflow);
    end if;

    // Local energy balance
    thermalPort.ports[i,1].Q_flow = convection.ht[i,1]*N_pipes*geometry.A_ht*
      (thermalPort.ports[i,1].T - Medium.temperature(state[i,1]));

    rho[i,1] = Medium.density(state[i,1]);
    u[i,1] = inlet.m_flow/(rho[i,1]*N_pipes*geometry.A_cs);

    // Non-dimensional numbers
    Re[i,1] = rho[i,1]*u[i,1]*geometry.Dh/Medium.dynamicViscosity(state[i,1]);
    Pr[i,1] = Medium.specificHeatCapacityCp(state[i,1])*
      Medium.dynamicViscosity(state[i,1])/
      Medium.thermalConductivity(state[i,1]);

    // Local pressure drop
    if DP_opt == Choices.PDropOpt.fixed then
      dP_cv[i,1] = dP_fixed/N_cv;
    elseif DP_opt == Choices.PDropOpt.correlation then
      dP_cv[i,1] = 0.5*homotopy(friction.f[i,1], 0.01)*rho[i,1]*u[i,1]^2/
        geometry.Dh*geometry.L/N_cv;
    elseif DP_opt == Choices.PDropOpt.linear then
      dP_cv[i,1] = Rh*inlet.m_flow/N_cv;
    else
      dP_cv[i,1] = 0;
    end if;
  end for;

  // Mass balance
  inlet.m_flow + outlet.m_flow = 0;
  G = abs(inlet.m_flow)/(N_pipes*geometry.A_cs);

  // Independent composition mass balances
  inlet.Xi_outflow = inStream(outlet.Xi_outflow);
  outlet.Xi_outflow = inStream(inlet.Xi_outflow);

  // Energy balance
  outlet.h_outflow = inStream(inlet.h_outflow) +
    sum(thermalPort.ports.Q_flow)/inlet.m_flow;
  inlet.h_outflow = inStream(outlet.h_outflow) +
    sum(thermalPort.ports.Q_flow)/inlet.m_flow;

  // Pressure drop
  dP = sum(dP_cv);
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
<p>Model of the flow through a circular pipe.</p>
<p>The convective heat transfer coefficient and the friction factor can be fixed by the user or computed with semi-empirical correlations.</p>
<p>The conductive heat transfer through the solid walls is not included in this model and must be treated separately.</p>
<p>The model can be used to reproduce the flow through many tubes in parallel. In that case, the mass flow rate is split equally among the different tubes.</p>
</html>"));
end CircularPipe;
