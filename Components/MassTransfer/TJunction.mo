within DynTherM.Components.MassTransfer;
model TJunction "T-junction with mass and energy accumulation"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  parameter Volume V "Junction volume";

  // Initialization
  parameter Pressure P_start=101325 "Pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start=288.15 "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X "Start gas composition" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature=false "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  Mass M "Total mass";
  InternalEnergy E "Total internal energy";
  Pressure P(start=P_start) "Pressure";
  Temperature T(start=T_start) "Temperature";
  MassFraction X[Medium.nX](start=X_start) "Mass fractions";
  Medium.ThermodynamicState state "Thermodynamic state";

  CustomInterfaces.ZeroDimensional.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0)) annotation (Placement(transformation(extent={{-120,-60},{-80,
            -20}}, rotation=0), iconTransformation(extent={{-110,-10},{-90,10}})));
  CustomInterfaces.ZeroDimensional.FluidPort_B outletParallel(
    redeclare package Medium = Medium,
     m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0)) annotation (Placement(transformation(extent={{80,-20},{120,20}},
          rotation=0), iconTransformation(extent={{90,-10},{110,10}})));
  CustomInterfaces.ZeroDimensional.FluidPort_B outletOrthogonal(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0)) annotation (Placement(transformation(extent={{-120,20},{-80,
            60}}, rotation=0), iconTransformation(extent={{-10,90},{10,110}})));

equation
  state = Medium.setState_pTX(P, T, X);

  // Conservation equations
  M = Medium.density(state)*V "Gas mass";
  E = M*Medium.specificInternalEnergy(state) "Gas internal energy";
  der(M) = inlet.m_flow + outletOrthogonal.m_flow + outletParallel.m_flow
    "Mass balance";
  der(E) = inlet.m_flow*actualStream(inlet.h_outflow) +
    outletOrthogonal.m_flow*actualStream(outletOrthogonal.h_outflow) +
    outletParallel.m_flow*actualStream(outletParallel.h_outflow) "Energy balance";

  for j in 1:Medium.nX loop
    M*der(X[j]) = inlet.m_flow*(actualStream(inlet.Xi_outflow[j]) - X[j]) +
      outletOrthogonal.m_flow*(actualStream(outletOrthogonal.Xi_outflow[j]) - X[j]) +
      outletParallel.m_flow*(actualStream(outletParallel.Xi_outflow[j]) - X[j])
      "Independent component mass balance";
  end for;

  // Boundary conditions
  inlet.P = P;
  inlet.h_outflow = Medium.specificEnthalpy(state);
  inlet.Xi_outflow = X;
  outletOrthogonal.P = P;
  outletOrthogonal.h_outflow = Medium.specificEnthalpy(state);
  outletOrthogonal.Xi_outflow = X;
  outletParallel.P = P;
  outletParallel.h_outflow = Medium.specificEnthalpy(state);
  outletParallel.Xi_outflow = X;

  // Only one connection allowed to a port to avoid unwanted ideal mixing
  assert(cardinality(inlet) <= 1, "inlet port can at most be connected to one component");
  assert(cardinality(outletOrthogonal) <= 1, "inlet port can at most be connected to one component");
  assert(cardinality(outletParallel) <= 1, "inlet port can at most be connected to one component");

initial equation
  // Initial conditions
  if initOpt == Choices.InitOpt.noInit then
    // do nothing
  elseif initOpt == Choices.InitOpt.fixedState then
    if not noInitialPressure then
      P = P_start;
    end if;
    if not noInitialTemperature then
      T = T_start;
    end if;
    X = X_start;
  elseif initOpt == Choices.InitOpt.steadyState then
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
</html>"), Icon(graphics={                                                                                                                                                                                                        Line(origin={-60,40},    points={{-40,-20},
              {40,-20},{40,60}},                                                                                                                                                                                                        color={0,0,0}),
                                                                                                                                                                                                        Line(origin={60,40},     points={{40,-20},
              {-40,-20},{-40,60}},                                                                                                                                                                                                      color={0,0,0}),
                                                                                                                                                                                                        Line(origin={-60,60},    points={{-40,-20},
              {20,-20},{20,40}},                                                                                                                                                                                                        color={0,0,0}),
                                                                                                                                                                                                        Line(origin={60,60},     points={{40,-20},
              {-20,-20},{-20,40}},                                                                                                                                                                                                      color={0,0,0}),
        Line(
          points={{-100,40},{-100,-40}},
          color={0,0,0}),
        Line(
          points={{0,40},{0,-40}},
          color={0,0,0},
          origin={0,100},
          rotation=-90),
        Line(
          points={{0,160},{0,-40}},
          color={0,0,0},
          origin={-60,-20},
          rotation=-90),
        Line(
          points={{0,160},{0,-40}},
          color={0,0,0},
          origin={-60,-40},
          rotation=-90),
        Line(
          points={{100,40},{100,-40}},
          color={0,0,0})}));
end TJunction;
