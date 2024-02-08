within DynTherM.Components.MassTransfer;
model PlenumSimple "Plenum with heat and mass transfer"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  outer DynTherM.Components.Environment environment "Environmental properties";

  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  parameter DynTherM.Choices.InitOpt initOpt=environment.initOpt
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Volume V "Inner volume";
  parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.AbsolutePressure P_start=101325 "Pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.Temperature T_start=300 "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.MassFraction X_start[Medium.nX]=Medium.reference_X "Start gas composition" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature=false "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  Modelica.Units.SI.Mass M "Total mass";
  Modelica.Units.SI.InternalEnergy E "Total internal energy";
  Medium.AbsolutePressure P "Pressure";
  Medium.Temperature T "Temperature";
  Medium.MassFraction X[Medium.nX] "Mass fractions";
  Medium.ThermodynamicState state "Thermodynamic state";
  Modelica.Units.SI.Time Tr "Residence Time";

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
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a thermalPort annotation (Placement(transformation(extent={{-18,62},{18,98}}),
        iconTransformation(extent={{-10,80},{10,100}})));

equation
  state = Medium.setState_pTX(P, T, X);

  // Conservation equations
  M = Medium.density(state)*V "Gas mass";
  E = M*Medium.specificInternalEnergy(state) "Gas internal energy";
  der(M) = inlet.m_flow + outlet.m_flow;
  der(E) = inlet.m_flow*actualStream(inlet.h_outflow) +
    outlet.m_flow*actualStream(outlet.h_outflow) + thermalPort.Q_flow;

  for i in 1:Medium.nX loop
    M*der(X[i]) = inlet.m_flow*(actualStream(inlet.Xi_outflow[i]) - X[i]) +
      outlet.m_flow*(actualStream(outlet.Xi_outflow[i]) - X[i]);
  end for;

  // Boundary conditions
  inlet.h_outflow = Medium.specificEnthalpy(state);
  inlet.Xi_outflow = X;
  outlet.h_outflow = Medium.specificEnthalpy(state);
  outlet.Xi_outflow = X;
  inlet.P = P;
  outlet.P = P;
  thermalPort.T = T;

  Tr = noEvent(M/max(abs(-outlet.m_flow), Modelica.Constants.eps))
    "Residence time";

initial equation
  // Initial conditions
  if initOpt == DynTherM.Choices.InitOpt.noInit then
    // do nothing
  elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
    if not noInitialPressure then
      P = P_start;
    end if;
    if not noInitialTemperature then
      T = T_start;
    end if;
    X = X_start;
  elseif initOpt == DynTherM.Choices.InitOpt.steadyState then
    if not noInitialPressure then
      der(P) = 0;
    end if;
    if not noInitialTemperature then
      der(T) = 0;
    end if;
    der(X) = zeros(Medium.nX);
  else
    assert(false, "Unsupported initialisation option");
  end if;
  annotation (
    Documentation(info="<html>
<p>Reference&nbsp;(for&nbsp;metabolic&nbsp;heat&nbsp;load&nbsp;calculations):</p>
<p>[1] C.&nbsp;Giaconia,&nbsp;et&nbsp;al.&nbsp;Air&nbsp;Quality&nbsp;and&nbsp;Relative&nbsp;Humidity&nbsp;in&nbsp;Commercial&nbsp;Aircraft:</p>
<p>[2] An&nbsp;Experimental&nbsp;Investigation&nbsp;on&nbsp;Short-Haul&nbsp;Domestic&nbsp;Flights,&nbsp;2013.</p>
<p>[3] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;Fundamentals,&nbsp;chapter&nbsp;9.</p>
<p>Hp:&nbsp;the&nbsp;occupants&nbsp;are&nbsp;always&nbsp;in&nbsp;thermal&nbsp;equilibrium,&nbsp;i.e.&nbsp;all&nbsp;the&nbsp;heat&nbsp;they generate&nbsp;is&nbsp;transferred&nbsp;to&nbsp;the&nbsp;environemnt&nbsp;as&nbsp;latent&nbsp;and&nbsp;sensible&nbsp;heat.</p>
</html>",
        revisions="<html>
<ul>
<li><i>30 May 2005</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Initialisation support added.</li>
<li><i>19 Nov 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Adapted to Modelica.Media
<li><i>5 Mar 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       First release.</li>
</ul>
</html>"),
    Icon(graphics={          Ellipse(
          extent={{-90,-90},{90,90}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5)}));
end PlenumSimple;
