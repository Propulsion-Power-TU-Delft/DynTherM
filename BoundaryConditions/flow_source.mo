within DynTherM.BoundaryConditions;
model flow_source "Flow rate source"
  outer DynTherM.Components.Environment environment "Environmental properties";
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  parameter Medium.AbsolutePressure P_nom=101325 "Nominal pressure";
  parameter Medium.Temperature T_nom=300 "Nominal temperature" annotation(Dialog(enable=not use_in_T and not use_di_T));
  parameter Medium.MassFraction X_nom[Medium.nX]=Medium.reference_X "Nominal mass fractions" annotation(Dialog(enable=not use_in_Xw and not use_di_Xw));
  parameter Medium.MassFlowRate massFlow_nom=1 "Nominal mass flowrate" annotation(Dialog(enable=not use_in_massFlow and not use_di_massFlow));
  parameter DynTherM.CustomUnits.HydraulicConductance G=0
    "HydraulicConductance";
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";

  // External Inputs
  parameter Boolean use_in_massFlow = false "Use connector input for the nominal flow rate" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));
  parameter Boolean use_in_T = false "Use connector input for the temperature" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));

  // Direct Inputs
  parameter Boolean use_di_massFlow=false "Use text-based defined mass flow rate" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
  parameter Boolean use_di_T=false "Use text-based defined temperature" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
  parameter Boolean use_di_X=false "Use text-based defined composition" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
  input Medium.MassFlowRate massFlow_di=massFlow_nom "Mass flow" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_massFlow));
  input Medium.Temperature T_di=T_nom "Temperature" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_T));
  input Medium.MassFraction X_di[Medium.nX]=X_nom "Water mass fraction" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_X));

  Medium.MassFlowRate massFlow(start=massFlow_nom) "Mass flow rate";
  Medium.AbsolutePressure P(start=P_nom) "Pressure";
  Medium.Temperature T(start=T_nom) "Temperature";
  Medium.MassFraction X[Medium.nX](start=X_nom) "Mass fractions";
  Medium.ThermodynamicState state "Thermodynamic state";

  DynTherM.CustomInterfaces.FluidPort_B outlet(redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0)) annotation (Placement(
        transformation(extent={{80,-20},{120,20}}, rotation=0),
        iconTransformation(extent={{90,-10},{110,10}})));
  Modelica.Blocks.Interfaces.RealInput in_massFlow if use_in_massFlow annotation (Placement(
        transformation(
        origin={-60,50},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,70})));
  Modelica.Blocks.Interfaces.RealInput in_T if use_in_T annotation (Placement(
        transformation(
        origin={-20,70},
        extent={{10,-10},{-10,10}},
        rotation=90), iconTransformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-20,70})));

protected
  Modelica.Blocks.Interfaces.RealInput in_massFlow_internal;
  Modelica.Blocks.Interfaces.RealInput in_T_internal;

equation
  state = Medium.setState_pTX(P, T, X);

  if G > 0 then
    outlet.m_flow = -massFlow + (outlet.P - P_nom)*G;
  else
    outlet.m_flow = -massFlow;
  end if;

  massFlow = in_massFlow_internal;
  if not use_in_massFlow and not use_di_massFlow then
    in_massFlow_internal = massFlow_nom "Flow rate set by parameter";
  elseif use_di_massFlow and not use_in_massFlow then
    in_massFlow_internal = massFlow_di "Flow rate set by direct inputs";
  end if;

  T = in_T_internal;
  if not use_in_T and not use_di_T then
    in_T_internal = T_nom "Temperature set by parameter";
  elseif use_di_T and not use_in_T then
    in_T_internal = T_di "Temperature set by direct inputs";
  end if;

  if use_di_X then
    X = X_di "Composition set by direct inputs";
  else
    X = X_nom "Composition set by parameter";
  end if;

  outlet.P = P;
  outlet.h_outflow = Medium.specificEnthalpy(state);
  outlet.Xi_outflow = X;

  // Connect protected connectors to public conditional connectors
  connect(in_massFlow, in_massFlow_internal);
  connect(in_T, in_T_internal);

  annotation (Documentation(info="<html>
<p>The actual gas used in the component is determined by the replaceable <span style=\"font-family: Courier New;\">Medium</span> package.</p>
<p>The source mass flow rate, temperature and mass fraction can be either specified as parameter, input or wired from input blocks.</p>
<p>Model adapted from <span style=\"font-family: Courier New;\">ThermoPower</span> library by Francesco Casella.</p>
</html>",
        revisions="<html>
</html>"),
         Icon(graphics={
        Rectangle(
          extent={{-100,60},{90,-60}},
          lineColor={128,128,128},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-40,-40},{40,0},{-40,40},{0,0},{-40,-40}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}));
end flow_source;
