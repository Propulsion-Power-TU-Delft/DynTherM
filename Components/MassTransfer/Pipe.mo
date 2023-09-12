within DynTherM.Components.MassTransfer;
model Pipe
  "Pipe model: pressure drop computed with Darcy-Weisbach equation with external heat input."
  outer DynTherM.Components.Environment environment "Environmental properties";
  package Medium = Modelica.Media.Air.MoistAir;
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  parameter DynTherM.Choices.PDropOpt option
    "Select the type of pressure drop to impose";
  parameter DynTherM.CustomUnits.HydraulicResistance R=0 "Hydraulic Resistance"
    annotation (Dialog(enable=option == Choices.PDropOpt.linear));
  parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure P_start=101325
    "Pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature T_start=300
    "Temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.MassFraction X_start[2]={0,1}
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Real csi_start=0.01 "Friction coefficient - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Velocity u_start=20
    "Flow velocity in the pipe - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure dP_start=100
    "Pressure drop - start value" annotation (Dialog(tab="Initialization",
        enable=option <> Choices.PDropOpt.fixed));
  parameter Modelica.Units.SI.Pressure dP_fixed=0 "Fixed pressure drop" annotation (Dialog(enable=option == Choices.PDropOpt.fixed));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Length L=0 "Pipe length" annotation (Dialog(enable=option == Choices.PDropOpt.darcyWeisbach));
  parameter Modelica.Units.SI.Length D=0 "Pipe diameter" annotation (Dialog(enable=option == Choices.PDropOpt.darcyWeisbach));
  parameter Modelica.Units.SI.Length Roughness=0.015*10^(-3) "Pipe roughness" annotation (Dialog(enable=option == Choices.PDropOpt.darcyWeisbach));

  Modelica.Units.SI.ReynoldsNumber Re "Reynolds number";
  Modelica.Units.SI.PrandtlNumber Pr "Prandtl number";
  Modelica.Units.SI.Velocity u(start=u_start) "Flow velocity in the pipe";
  Real csi(start=csi_start) "Friction coefficient";
  Modelica.Units.SI.Pressure dP(start=dP_start) "Pressure drop";
  Medium.ThermodynamicState state "Average thermodynamic state";
  Medium.ThermodynamicState state_inlet "Inlet state";
  Medium.ThermodynamicState state_outlet "Outlet state";

  DynTherM.CustomInterfaces.FluidPort_A inlet(
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-120,
            -20},{-80,20}}, rotation=0), iconTransformation(extent={{-110,-10},
            {-90,10}})));
  DynTherM.CustomInterfaces.FluidPort_B outlet(
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=
          -m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{80,
            -20},{120,20}}, rotation=0), iconTransformation(extent={{90,-10},{
            110,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a thermalPort
    annotation (Placement(transformation(extent={{-10,28},{10,48}})));
equation
  state_inlet = Medium.setState_phX(inlet.P, actualStream(inlet.h_outflow), actualStream(inlet.Xi_outflow));
  state_outlet = Medium.setState_phX(outlet.P, actualStream(outlet.h_outflow), actualStream(outlet.Xi_outflow));
  state = Medium.setState_phX((inlet.P + outlet.P)/2, (inStream(inlet.h_outflow) + inStream(outlet.h_outflow))/2, inlet.Xi_outflow);

  // Mass balance
  inlet.m_flow + outlet.m_flow = 0;

  // Independent composition mass balances
  inlet.Xi_outflow = inStream(outlet.Xi_outflow);
  inStream(inlet.Xi_outflow) = outlet.Xi_outflow;

  // Energy balance
  actualStream(inlet.h_outflow)*inlet.m_flow + thermalPort.Q_flow + actualStream(outlet.h_outflow)*outlet.m_flow = 0;
  inlet.h_outflow = outlet.h_outflow;

  // Flow characteristics
  if option == Choices.PDropOpt.fixed then
    dP = dP_fixed;
    csi = 0;
    u = 0;
    Re = 0;
    Pr = 0;
  elseif option == Choices.PDropOpt.darcyWeisbach then
    // Darcy-Weisbach
    u = inlet.m_flow/(Medium.density(state)*((environment.pi*D^2)/4));
    Re = Medium.density(state)*u*D/Medium.dynamicViscosity(state);
    Pr = Medium.specificHeatCapacityCp(state)*Medium.dynamicViscosity(state)/
      Medium.thermalConductivity(state);
    if Re < 2000 then                    // Laminar
      csi = 64/Re;
    elseif Re > 4000 and Re < 10^4 then  // Transition
      csi = 0.316/(Re^0.25);
    else                                 // Turbulent
      1/sqrt(csi) = -2*log10(Roughness/(3.7*D) + 2.51/(Re*sqrt(csi)));
    end if;
    dP = 0.5*csi*L/D*Medium.density(state)*u^2;
  elseif option == Choices.PDropOpt.linear then
    dP = R*inlet.m_flow;
    csi = 0;
    u = 0;
    Re = 0;
    Pr = 0;
  else
    dP = 0;
    csi = 0;
    u = 0;
    Re = 0;
    Pr = 0;
  end if;

  inlet.P - outlet.P = dP;

  thermalPort.T = if inlet.m_flow > 0 then Medium.temperature(state_inlet) else Medium.temperature(state_outlet);
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
end Pipe;
