within DynTherM.BoundaryConditions;
model flow_source_distributed "Distributed flow rate source"
  outer DynTherM.Components.Environment environment "Environmental properties";
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  parameter Integer N(min=1) "Number of ports";
  parameter Boolean use_massFlow=true "Use text-based defined mass flow rate" annotation(choices(checkBox=true));
  parameter Boolean use_T=true "Use text-based defined temperature" annotation(choices(checkBox=true));
  parameter Boolean use_X=true "Use text-based defined composition" annotation(choices(checkBox=true));
  input Medium.MassFlowRate massFlow_di[N]=ones(N) "Mass flow" annotation(Dialog(enable=use_di_massFlow));
  input Medium.Temperature T_di[N]=288.15*ones(N) "Temperature" annotation(Dialog(enable=use_di_T));
  input Medium.MassFraction X_di[Medium.nX]=Medium.reference_X "Mass fractions" annotation(Dialog(enable=use_di_X));
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";

  Medium.MassFlowRate massFlow[N](start=ones(N)) "Mass flow rate";
  Medium.AbsolutePressure P[N](start=101325*ones(N)) "Pressure";
  Medium.Temperature T[N](start=288.15*ones(N)) "Temperature";
  Medium.MassFraction X[Medium.nX](start=Medium.reference_X) "Mass fractions";
  Medium.ThermodynamicState state[N] "Thermodynamic state";

  CustomInterfaces.DistributedFluidPort_B outlet(
    redeclare package Medium = Medium,
    N=N) annotation (
      Placement(transformation(extent={{98,-2},{120,20}}), iconTransformation(
        extent={{-40,-35},{40,35}},
        rotation=-90,
        origin={97,7.10543e-15})));

equation

  if use_massFlow then
    massFlow = massFlow_di;
  end if;

  if use_T then
    T = T_di;
  end if;

  if use_X then
    X = X_di;
  end if;

  outlet.ports.m_flow + massFlow = zeros(N);
  outlet.ports.P = P;

  for i in 1:N loop
    state[i] = Medium.setState_pTX(P[i], T[i], X);
    outlet.ports[i].h_outflow = Medium.specificEnthalpy(state[i]);
    outlet.ports[i].Xi_outflow = X;
  end for;

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
end flow_source_distributed;
