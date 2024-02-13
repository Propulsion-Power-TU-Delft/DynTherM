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
<p>Flow source model featuring a distributed flow port.</p>
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
end flow_source_distributed;
