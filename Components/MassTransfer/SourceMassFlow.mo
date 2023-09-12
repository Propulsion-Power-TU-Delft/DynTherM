within ThermalManagement.Components.MassTransfer;
model SourceMassFlow "Flow rate source for gas flows"
  package Medium = Modelica.Media.Air.MoistAir;
  outer ThermalManagement.Components.Environment environment "Environmental properties";
  parameter Medium.AbsolutePressure P_nom=101325 "Nominal pressure";
  parameter Medium.Temperature T_nom=300 "Nominal temperature" annotation(Dialog(enable=not use_in_T and not use_di_T));
  parameter Medium.MassFraction Xw_nom=0 "Nominal water mass fraction" annotation(Dialog(enable=not use_in_Xw and not use_di_Xw));
  parameter Medium.MassFlowRate massFlow_nom=0 "Nominal mass flowrate" annotation(Dialog(enable=not use_in_massFlow and not use_di_massFlow));
  parameter ThermalManagement.CustomUnits.HydraulicConductance G=0 "HydraulicConductance";
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  // External Inputs
  parameter Boolean use_in_massFlow = false "Use connector input for the nominal flow rate" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));
  parameter Boolean use_in_T = false "Use connector input for the temperature" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));
  parameter Boolean use_in_Xw = false "Use connector input for the composition" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));
  // Direct Inputs
  parameter Boolean use_di_massFlow=false "Use text-based defined mass flow rate" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
  parameter Boolean use_di_T=false "Use text-based defined temperature" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
  parameter Boolean use_di_Xw=false "Use text-based defined composition" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
  input Medium.MassFlowRate massFlow_di=massFlow_nom "Mass flow" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_massFlow));
  input Medium.Temperature T_di=T_nom "Temperature" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_T));
  input Medium.MassFraction Xw_di=Xw_nom "Water mass fraction" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_Xw));

  Medium.MassFlowRate massFlow "Mass flow rate";
  Medium.AbsolutePressure P(start=P_nom) "Pressure";
  Medium.Temperature T(start=T_nom) "Temperature";
  Medium.MassFraction X[2](start={Xw_nom, 1 - Xw_nom}) "Mass fractions";
  Real phi "Relative humidity";
  Medium.ThermodynamicState state "Thermodynamic state";
  ThermalManagement.CustomInterfaces.FluidPort_B outlet(
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf
    else 0)) annotation (
      Placement(transformation(extent={{80,-20},{120,20}}, rotation=0),
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
  Modelica.Blocks.Interfaces.RealInput in_Xw if use_in_Xw annotation (Placement(
        transformation(
        origin={60,50},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,70})));
protected
  Modelica.Blocks.Interfaces.RealInput in_massFlow_internal;
  Modelica.Blocks.Interfaces.RealInput in_T_internal;
  Modelica.Blocks.Interfaces.RealInput in_Xw_internal;

equation
  state = Medium.setState_pTX(P, T, X);
  phi = Medium.relativeHumidity(state);

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

  X = {in_Xw_internal, 1 - in_Xw_internal};
  if not use_in_Xw and not use_di_Xw then
    in_Xw_internal = Xw_nom "Composition set by parameter";
  elseif use_di_Xw and not use_in_Xw then
    in_Xw_internal = Xw_di "Composition set by direct inputs";
  end if;

  outlet.P = P;
  outlet.h_outflow = Medium.specificEnthalpy(state);
  outlet.Xi_outflow = X;

  // Connect protected connectors to public conditional connectors
  connect(in_massFlow, in_massFlow_internal);
  connect(in_T, in_T_internal);
  connect(in_Xw, in_Xw_internal);

  annotation (Documentation(info="<html>
<p><b>Modelling options</b></p>
<p>The actual gas used in the component is determined by the replaceable <tt>Medium</tt> package. In the case of multiple component, variable composition gases, the nominal gas composition is given by <tt>Xnom</tt>,whose default value is <tt>Medium.reference_X</tt> .
<p>If <tt>G</tt> is set to zero, the flowrate source is ideal; otherwise, the outgoing flowrate decreases proportionally to the outlet pressure.</p>
<p>If the <tt>in_w0</tt> connector is wired, then the source massflowrate is given by the corresponding signal, otherwise it is fixed to <tt>w0</tt>.</p>
<p>If the <tt>in_T</tt> connector is wired, then the source temperature is given by the corresponding signal, otherwise it is fixed to <tt>T</tt>.</p>
<p>If the <tt>in_X</tt> connector is wired, then the source massfraction is given by the corresponding signal, otherwise it is fixed to <tt>Xnom</tt>.</p>
</html>",
        revisions="<html>
<ul>
<li><i>19 Nov 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Removed <tt>w0fix</tt> and <tt>Tfix</tt> and <tt>Xfix</tt>; the connection of external signals is now detected automatically.</li> <br> Adapted to Modelica.Media
<li><i>1 Oct 2003</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       First release.</li>
</ul>
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
end SourceMassFlow;
