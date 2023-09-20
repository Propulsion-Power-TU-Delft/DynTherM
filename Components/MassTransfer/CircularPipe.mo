within DynTherM.Components.MassTransfer;
model CircularPipe "Model of pipe with arbitrary cross-section"

  outer DynTherM.Components.Environment environment "Environmental properties";
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Options and initialization
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  parameter DynTherM.Choices.PDropOpt DP_opt
    "Select the type of pressure drop to impose";
  parameter DynTherM.CustomUnits.HydraulicResistance Rh=1
    "Hydraulic Resistance" annotation (Dialog(enable=DP_opt == Choices.PDropOpt.linear));
  parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure P_start=101325
    "Pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature T_start=300
    "Temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Velocity u_start=20
    "Flow velocity in the pipe - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure dP_start=100
    "Pressure drop - start value" annotation (Dialog(tab="Initialization",
        enable=option <> Choices.PDropOpt.fixed));
  parameter Modelica.Units.SI.Pressure dP_fixed=0 "Fixed pressure drop" annotation (Dialog(enable=DP_opt == Choices.PDropOpt.fixed));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.ReynoldsNumber Re_start=20e3 "Reynolds number - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.PrandtlNumber Pr_start=1.5 "Prandtl number - start value" annotation (Dialog(tab="Initialization"));

  // Geometry
  parameter Modelica.Units.SI.Length L "Length" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length D "Pipe diameter" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length Roughness=0.015*10^(-3) "Pipe roughness" annotation (Dialog(tab="Geometry"));

  model GEO =
    DynTherM.Components.MassTransfer.PipeGeometry.Circular (
      L=L, D=D) annotation (choicesAllMatching=true);

  // Heat transfer
  model HTC =
    DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.DittusBoelter
      (redeclare package Medium=Medium,
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
      Dh=geometry.Dh,
      Re=Re,
      Roughness=Roughness) annotation (choicesAllMatching=true);

  HTC convection;
  DPC friction;
  GEO geometry;

  Modelica.Units.SI.ReynoldsNumber Re(start=Re_start) "Reynolds number";
  Modelica.Units.SI.PrandtlNumber Pr(start=Pr_start) "Prandtl number";
  Modelica.Units.SI.Velocity u(start=u_start) "Flow velocity in the pipe";
  Modelica.Units.SI.Pressure dP(start=dP_start) "Pressure drop";
  Medium.ThermodynamicState state "Average thermodynamic state";
  Medium.ThermodynamicState state_inlet "Inlet state";
  Medium.ThermodynamicState state_outlet "Outlet state";

  DynTherM.CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-120,
            -20},{-80,20}}, rotation=0), iconTransformation(extent={{-110,-10},
            {-90,10}})));
  DynTherM.CustomInterfaces.FluidPort_B outlet(
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
  state_inlet = Medium.setState_phX(inlet.P, actualStream(inlet.h_outflow), actualStream(inlet.Xi_outflow));
  state_outlet = Medium.setState_phX(outlet.P, actualStream(outlet.h_outflow), actualStream(outlet.Xi_outflow));
  state = Medium.setState_phX((inlet.P + outlet.P)/2, (actualStream(inlet.h_outflow) + actualStream(outlet.h_outflow))/2, inlet.Xi_outflow);

  // Mass balance
  inlet.m_flow + outlet.m_flow = 0;

  // Independent composition mass balances
  inlet.Xi_outflow = inStream(outlet.Xi_outflow);
  outlet.Xi_outflow = inStream(inlet.Xi_outflow);

  // Energy balance
  outlet.h_outflow = inStream(inlet.h_outflow) + thermalPort.Q_flow/inlet.m_flow;
  inlet.h_outflow = inStream(outlet.h_outflow) + thermalPort.Q_flow/inlet.m_flow;
  thermalPort.Q_flow = convection.ht*geometry.A_ht*(thermalPort.T - Medium.temperature(state));

  // Non-dimensional numbers
  u = inlet.m_flow/(Medium.density(state)*geometry.A_cs);
  Re = Medium.density(state)*u*geometry.Dh/Medium.dynamicViscosity(state);
  Pr = Medium.specificHeatCapacityCp(state)*Medium.dynamicViscosity(state)/
    Medium.thermalConductivity(state);

  // Pressure drop
  if DP_opt == Choices.PDropOpt.fixed then
    dP = dP_fixed;
  elseif DP_opt == Choices.PDropOpt.correlation then
    dP = 0.5*homotopy(friction.f, 0.01)*Medium.density(state)*u^2/geometry.Dh*geometry.L;
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
        coordinateSystem(preserveAspectRatio=false)));
end CircularPipe;
