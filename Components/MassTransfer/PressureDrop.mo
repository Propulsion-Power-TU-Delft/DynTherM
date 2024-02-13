within DynTherM.Components.MassTransfer;
model PressureDrop "Simple concentrated pressure drop, featuring a constant or linear characteristic"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  outer DynTherM.Components.Environment environment "Environmental properties";

  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  parameter DynTherM.Choices.PDropOpt option
    "Select the type of pressure drop to impose";
  parameter DynTherM.CustomUnits.HydraulicResistance R=0 "Hydraulic Resistance";
  parameter Modelica.Units.SI.Pressure dP_fixed=0 "Fixed pressure drop";
  parameter Modelica.Units.SI.Pressure dP_start=100
    "Pressure drop - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure P_start=101325
    "Pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature T_start=300
    "Temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  Modelica.Units.SI.Pressure dP(start=dP_start) "Pressure drop";

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
equation
  inlet.m_flow + outlet.m_flow = 0 "Mass balance";
  if inlet.m_flow > 0 then
    inlet.P - outlet.P = dP;
  else
    outlet.P - inlet.P = dP;
  end if;

  // Flow characteristics
  if option == DynTherM.Choices.PDropOpt.fixed then
    dP = dP_fixed;
  else
    dP = R*inlet.m_flow;
  end if;

  // Energy balance
  inlet.h_outflow = inStream(outlet.h_outflow);
  outlet.h_outflow = inStream(inlet.h_outflow);

  // Independent composition mass balances
  inlet.Xi_outflow = inStream(outlet.Xi_outflow);
  outlet.Xi_outflow = inStream(inlet.Xi_outflow);

  annotation (
    Documentation(info="<html>
<p>Model adapted from ThermoPower library by Francesco Casella.</p>
</html>",
        revisions="<html>
</html>"), Icon(graphics={Rectangle(
          extent={{-90,40},{90,-40}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-70,24},{74,-26}},
          lineColor={0,0,0},
          lineThickness=0.5,
          textString="P DROP")}));
end PressureDrop;
