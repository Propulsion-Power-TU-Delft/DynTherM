within DynTherM.Components.MassTransfer;
model Plenum
  "Plenum with heat and mass transfer + internal sensible and latent heat generation from occupants"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  outer Components.Environment environment "Environmental properties";

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  parameter Volume V "Inner volume";
  parameter Real N_occupants[3]={0,0,0}
    "Number of: passengers, cabin crew, pilots inside the plenum";
  input HeatFlowRate Q_int=0 "Internal heat load" annotation (Dialog(enable=true));
  parameter HeatFlowRate Q_sens_fixed[3]={0,0,0}
    "Fixed value of sensible heat from the occupants" annotation (Dialog(enable=fixed_Q));
  parameter HeatFlowRate Q_lat_fixed[3]={0,0,0}
    "Fixed value of latent heat from the occupants" annotation (Dialog(enable=fixed_Q));
  parameter MassFlowRate m_H2O_fixed[3]={0,0,0}
    "Fixed value of water vapor emitted by the occupants" annotation (Dialog(enable=fixed_Q));

  // Initialization
  parameter MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325 "Pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start=288.15 "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X "Start gas composition" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature=false "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean fixed_Q=false "= true to fix the value of sensible and latent heat from the occupants";

  Mass M "Total mass";
  InternalEnergy E "Total internal energy";
  Pressure P(start=P_start) "Pressure";
  Temperature T(start=T_start) "Temperature";
  MassFraction X[Medium.nX](start=X_start) "Mass fractions";
  Medium.ThermodynamicState state "Thermodynamic state";
  Time Tr "Residence Time";
  Pressure Pv "Pressure of water vapor";
  Temperature T_skin[3] "Skin temperature of the occupants";
  DensityOfHeatFlowRate E_dif[3]
    "Rate of heat loss due to water diffusion through the skin";
  DensityOfHeatFlowRate E_res[3]
    "Rate of heat loss due to respiration";
  DensityOfHeatFlowRate E_rsw[3]
    "Rate of heat loss due to sweating";
  DensityOfHeatFlowRate E_hb[3]
    "Rate of evaporative heat loss of the human body";
  MassFlowRate m_H2O[3]
    "Water vapor emitted by the occupants";
  HeatFlowRate Q_sens[3]
    "Sensible heat flow rate from occupants";
  HeatFlowRate Q_lat[3]
    "Latent heat flow rate from occupants";

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
  der(M) = inlet.m_flow + outlet.m_flow + sum(m_H2O) "Mass balance";
  der(E) = inlet.m_flow*actualStream(inlet.h_outflow) +
    outlet.m_flow*actualStream(outlet.h_outflow) + thermalPort.Q_flow +
    sum(Q_sens) + sum(Q_lat) + Q_int "Energy balance";
  M*der(X[1]) = inlet.m_flow*(actualStream(inlet.Xi_outflow[1]) - X[1]) +
    outlet.m_flow*(actualStream(outlet.Xi_outflow[1]) - X[1]) +
    sum(m_H2O)*(1 - X[1]) "Water mass balance";
  M*der(X[2]) = inlet.m_flow*(actualStream(inlet.Xi_outflow[2]) - X[2]) +
    outlet.m_flow*(actualStream(outlet.Xi_outflow[2]) - X[2]) -
    sum(m_H2O)*X[2] "Dry air mass balance";

  // Metabolic heat
  Pv = Medium.saturationPressure(T)*Medium.relativeHumidity(state);
  if fixed_Q then
    // Fixed values of latent, sensible heat and water vapor released by the occupants
    Q_sens = Q_sens_fixed;
    m_H2O = m_H2O_fixed;
    Q_lat = Q_lat_fixed;
    T_skin = zeros(3);
    E_dif = zeros(3);
    E_res = zeros(3);
    E_rsw = zeros(3);
    E_hb = E_dif + E_res + E_rsw;
  else
    // Direct computation of latent, sensible heat and water vapor released by the occupants
    for j in 1:3 loop
      T_skin[j] = 273.15 + (37.5 - 0.0275*environment.M_hb[j]);
      E_dif[j] = 3.05*(5.73 - 0.007*environment.M_hb[j] - Pv/1000);
      E_res[j] = 0.0173*environment.M_hb[j]*(5.87 - Pv/1000);
      E_rsw[j] = 0.42*(environment.M_hb[j] - 58.15);
      E_hb[j] = E_dif[j] + E_res[j] + E_rsw[j];
      m_H2O[j] = N_occupants[j]*E_hb[j]*environment.S_hb/
        (Medium.enthalpyOfCondensingGas(T) - Medium.enthalpyOfWater(T_skin[j]));
      Q_sens[j] = N_occupants[j]*(environment.M_hb[j] - E_hb[j])*environment.S_hb;
      Q_lat[j] = m_H2O[j]*(Medium.enthalpyOfCondensingGas(T) - Medium.enthalpyOfWater(T_skin[j]));
    end for;
  end if;

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
    der(X) = zeros(2);
  else
    assert(false, "Unsupported initialisation option");
  end if;
  annotation (
    Documentation(info="<html>
<p>Plenum model featuring also internal heat and water vapor generation by occupants.</p>
<p><br><b>Assumption:</b> the occupants are always in thermal equilibrium, i.e. all the heat they generate is transferred to the environemnt as latent and sensible heat.</p>
<p><br><b>References:</b></p>
<p>[1] C.&nbsp;Giaconia,&nbsp;et&nbsp;al.&nbsp;Air&nbsp;Quality&nbsp;and&nbsp;Relative&nbsp;Humidity&nbsp;in&nbsp;Commercial&nbsp;Aircraft:</p>
<p>[2] An&nbsp;Experimental&nbsp;Investigation&nbsp;on&nbsp;Short-Haul&nbsp;Domestic&nbsp;Flights,&nbsp;2013.</p>
<p>[3] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;Fundamentals,&nbsp;chapter&nbsp;9.&nbsp;</p>
</html>",
        revisions="<html>
</html>"),
    Icon(graphics={          Ellipse(
          extent={{-90,-90},{90,90}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5)}));
end Plenum;
