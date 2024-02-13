within DynTherM.Components.MassTransfer;
model Mixer "Mixer of two streams with heat and mass transfer"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  parameter Modelica.Units.SI.Volume V "Inner volume";
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  outer DynTherM.Components.Environment environment "Environmental properties";
  parameter Medium.AbsolutePressure P_start=101325 "Pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.Temperature T_start=300 "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.MassFraction X_start[Medium.nX]=Medium.reference_X "Start gas composition" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt=environment.initOpt
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature=false "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  Modelica.Units.SI.Mass M "Total mass";
  Modelica.Units.SI.InternalEnergy E "Total internal energy";
  Medium.AbsolutePressure P(start=P_start) "Pressure";
  Medium.Temperature T(start=T_start) "Temperature";
  Medium.MassFraction X[Medium.nX](start=X_start) "Mass fractions";
  Medium.ThermodynamicState state "Thermodynamic state";
  Modelica.Units.SI.Time Tr "Residence Time";

  DynTherM.CustomInterfaces.FluidPort_A inlet1(redeclare package Medium =
        Medium,                                m_flow(min=if allowFlowReversal
           then -Modelica.Constants.inf else 0)) annotation (Placement(
        transformation(extent={{-120,-60},{-80,-20}}, rotation=0),
        iconTransformation(extent={{-90,-50},{-70,-30}})));
  DynTherM.CustomInterfaces.FluidPort_B outlet(redeclare package Medium =
        Medium,                                m_flow(max=if allowFlowReversal
           then +Modelica.Constants.inf else 0)) annotation (Placement(
        transformation(extent={{80,-20},{120,20}}, rotation=0),
        iconTransformation(extent={{90,-10},{110,10}})));

  CustomInterfaces.FluidPort_A inlet2(redeclare package Medium = Medium,
                                      m_flow(min=if allowFlowReversal then -
          Modelica.Constants.inf else 0)) annotation (Placement(
        transformation(extent={{-120,20},{-80,60}}, rotation=0),
        iconTransformation(extent={{-90,30},{-70,50}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a thermalPort annotation (Placement(transformation(extent={{-18,62},
            {18,98}}),
        iconTransformation(extent={{-10,80},{10,100}})));
equation
  state = Medium.setState_pTX(P, T, X);

  // Conservation equations
  M = Medium.density(state)*V "Gas mass";
  E = M*Medium.specificInternalEnergy(state) "Gas internal energy";
  der(M) = inlet1.m_flow + inlet2.m_flow + outlet.m_flow "Mass balance";
  der(E) = inlet1.m_flow*actualStream(inlet1.h_outflow) +
    inlet2.m_flow*actualStream(inlet2.h_outflow) +
    outlet.m_flow*actualStream(outlet.h_outflow) +
    thermalPort.Q_flow "Energy balance";

  for j in 1:Medium.nX loop
    M*der(X[j]) = inlet1.m_flow*(actualStream(inlet1.Xi_outflow[j]) - X[j]) +
      inlet2.m_flow*(actualStream(inlet2.Xi_outflow[j]) - X[j]) +
      outlet.m_flow*(actualStream(outlet.Xi_outflow[j]) - X[j])
      "Independent component mass balance";
  end for;

  // Boundary conditions
  inlet1.P = P;
  inlet1.h_outflow = Medium.specificEnthalpy(state);
  inlet1.Xi_outflow = X;
  inlet2.P = P;
  inlet2.h_outflow = Medium.specificEnthalpy(state);
  inlet2.Xi_outflow = X;
  outlet.P = P;
  outlet.h_outflow = Medium.specificEnthalpy(state);
  outlet.Xi_outflow = X;
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
<p>Same as PlenumSimple model, but featuring two inlet ports to model the mixing of two incoming streams.</p>
<p>Model adapted from ThermoPower library by Francesco Casella.</p>
</html>",
        revisions="<html>
</html>"),
    Icon(graphics={          Ellipse(
          extent={{-90,-90},{90,90}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5)}));
end Mixer;
