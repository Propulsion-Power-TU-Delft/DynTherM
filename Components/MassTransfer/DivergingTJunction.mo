within DynTherM.Components.MassTransfer;
model DivergingTJunction "Diverging T-junction with circular cross section"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  // Geometry
  parameter Length D_in "Inlet diameter" annotation (Dialog(tab="Geometry"));
  parameter Length D_out_main "Outlet diameter - main branch" annotation (Dialog(tab="Geometry"));
  parameter Length D_out_side "Outlet diameter - side branch" annotation (Dialog(tab="Geometry"));
  parameter Length R_bend "Radius of curvature" annotation (Dialog(tab="Geometry"));
  parameter Volume V "Junction volume" annotation (Dialog(tab="Geometry"));

  // Initialization
  parameter Pressure P_start=101325 "Pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start=288.15 "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X "Start gas composition" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start=20 "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
  parameter Density rho_start=1 "Density - start value" annotation (Dialog(tab="Initialization"));
  parameter Real K_start=0.2 "Diverging loss coefficient - start value" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature=false "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  Area A_cs_in "Inlet cross sectional area";
  Area A_cs_out_main "Outlet cross sectional area - main branch";
  Area A_cs_out_side "Outlet cross sectional area - side branch";
  Mass M "Total mass";
  CustomUnits.MassFlux G_in "Inlet mass flux";
  CustomUnits.MassFlux G_out_main "Outlet mass flux - main branch";
  CustomUnits.MassFlux G_out_side "Outlet mass flux - side branch";
  Velocity u_in(start=u_start) "Inlet flow velocity";
  Velocity u_out_main(start=u_start) "Outlet flow velocity - main branch";
  Velocity u_out_side(start=u_start) "Outlet flow velocity - side branch";
  InternalEnergy E "Total internal energy";
  Pressure P(start=P_start) "Average pressure";
  Temperature T(start=T_start) "Average temperature";
  Density rho(start=rho_start) "Average density";
  MassFraction X[Medium.nX](start=X_start) "Mass fractions";
  Real K_main(start=K_start, min=0, max=20) "Diverging loss coefficient - main branch";
  Real K_side(start=K_start, min=0, max=20) "Diverging loss coefficient - side branch";
  Medium.ThermodynamicState state "Thermodynamic state";
  PressureDifference dP_out_main "Pressure drop in main branch";
  PressureDifference dP_out_side "Pressure drop in side branch";

  CustomInterfaces.ZeroDimensional.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0))
    "Inlet - main branch"                                                 annotation (Placement(transformation(extent={{-120,-60},{-80,
            -20}}, rotation=0), iconTransformation(extent={{-110,-10},{-90,10}})));
  CustomInterfaces.ZeroDimensional.FluidPort_B outlet_main(redeclare package
      Medium = Medium, m_flow(max=if allowFlowReversal then +Modelica.Constants.inf
           else 0)) "Outlet - main branch" annotation (Placement(transformation(
          extent={{80,-20},{120,20}}, rotation=0), iconTransformation(extent={{90,
            -10},{110,10}})));
  CustomInterfaces.ZeroDimensional.FluidPort_B outlet_side(redeclare package
      Medium = Medium, m_flow(max=if allowFlowReversal then +Modelica.Constants.inf
           else 0)) "Outlet - side branch" annotation (Placement(transformation(
          extent={{-120,20},{-80,60}}, rotation=0), iconTransformation(extent={{
            -10,90},{10,110}})));

protected
  Real K_star(start=0.2, min=0, max=20);

equation
  state = Medium.setState_pTX(P, T, X);

  A_cs_in = pi*D_in^2/4;
  A_cs_out_main = pi*D_out_main^2/4;
  A_cs_out_side = pi*D_out_side^2/4;

  rho = Medium.density(state);
  u_in = G_in/rho;
  u_out_main = G_out_main/rho;
  u_out_side = G_out_side/rho;

  // Losses
  K_main = 0.62 - 0.98*abs(inlet.m_flow/outlet_main.m_flow) +
    0.36*abs(inlet.m_flow/outlet_main.m_flow)^2 +
    0.03*abs(inlet.m_flow/outlet_main.m_flow)^6;
  K_side = (0.81 - 1.13*abs(inlet.m_flow/outlet_side.m_flow) +
    abs(inlet.m_flow/outlet_side.m_flow)^2)*(D_out_side/D_in)^4 +
    1.12*D_out_side/D_in - 1.08*(D_out_side/D_in)^3 + K_star;
  K_star = 0.57 - 1.07*sqrt(R_bend/D_out_side) -
    2.13*R_bend/D_out_side + 8.24*(R_bend/D_out_side)^(3/2) -
    8.48*(R_bend/D_out_side)^2 + 2.9*(R_bend/D_out_side)^(5/2);

  // Mass balance
  M = Medium.density(state)*V;
  G_in = abs(inlet.m_flow)/A_cs_in;
  G_out_main = abs(outlet_main.m_flow)/A_cs_out_main;
  G_out_side = abs(outlet_side.m_flow)/A_cs_out_side;
  der(M) = inlet.m_flow + outlet_side.m_flow + outlet_main.m_flow;

  // Momentum balance
  inlet.P = P;  // Hp: no pressure drop in main branch inlet for numerical stability
  P - outlet_main.P = homotopy(dP_out_main, 0);
  P - outlet_side.P = homotopy(dP_out_side, 0);
  dP_out_main = K_main*rho*u_out_main^2/2;
  dP_out_side = K_side*rho*u_out_side^2/2;

  // Energy balance
  E = M*Medium.specificInternalEnergy(state);
  der(E) =inlet.m_flow*actualStream(inlet.h_outflow) + outlet_side.m_flow*
    actualStream(outlet_side.h_outflow) + outlet_main.m_flow*actualStream(
    outlet_main.h_outflow);

  for j in 1:Medium.nX loop
    M*der(X[j]) =inlet.m_flow*(actualStream(inlet.Xi_outflow[j]) - X[j]) +
      outlet_side.m_flow*(actualStream(outlet_side.Xi_outflow[j]) - X[j]) +
      outlet_main.m_flow*(actualStream(outlet_main.Xi_outflow[j]) - X[j])
      "Independent component mass balance";
  end for;

  // Boundary conditions
  inlet.h_outflow = Medium.specificEnthalpy(state);
  inlet.Xi_outflow = X;
  outlet_side.h_outflow = Medium.specificEnthalpy(state);
  outlet_side.Xi_outflow = X;
  outlet_main.h_outflow = Medium.specificEnthalpy(state);
  outlet_main.Xi_outflow = X;

  // Only one connection allowed to a port to avoid unwanted ideal mixing
  assert(cardinality(inlet) <= 1, "inlet port can at most be connected to one component");
  assert(cardinality(outlet_side) <= 1, "inlet port can at most be connected to one component");
  assert(cardinality(outlet_main) <= 1, "inlet port can at most be connected to one component");

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
<h4>References:</h4>
<p>[1] Rennels, D. C., &amp; Hudson, H. M.&nbsp;Pipe flow: A practical and comprehensive guide. Hoboken, N.J: John Wiley &amp; Sons., 2012.</p>
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
end DivergingTJunction;
