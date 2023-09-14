within DynTherM.Systems.Helicopter.Subsystems;
model Evaporator
  "Ideal evaporator for air-side calculations based on temperature control, with two intake streams"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  parameter Modelica.Units.SI.MassFlowRate m_fan "Fan air through-flow"
    annotation (Dialog(group="Airflow"));
  parameter Medium.Temperature T_start_out=283.15 "Initial temperature of evaporator outflow" annotation(Dialog(tab="Initialisation"));
  parameter Medium.MassFraction X_start_out = 0.01 "Initial water mass ratio of evaporator outflow" annotation(Dialog(tab="Initialisation"));
  //parameter Medium.Temperature T_start_in=300   "Initial temperature of evaporator inflow" annotation(Dialog(tab="Initialisation"));
  //parameter Medium.MassFraction X_start_in = 0.01 "Initial water mass ratio of evaporator inflow" annotation(Dialog(tab="Initialisation"));
  parameter Real nu_fan = 0.7 "Fan mechanical efficiency";
  parameter Modelica.Units.SI.AbsolutePressure dP=0 "Fixed pressure drop across evaporator";

  // Heat Load
  Modelica.Units.SI.HeatFlowRate Q_lat "Latent heat power";
  Modelica.Units.SI.HeatFlowRate Q "Total heat duty";
  Modelica.Units.SI.HeatFlowRate Q_sens "Sensible heat power";
  Modelica.Units.SI.HeatFlowRate Q2
    "Alternate total heat duty used for comparison";
  Modelica.Units.SI.HeatFlowRate Q_sens_cp
    "Cp sensible heat power used for comparison";
  // Outlet Flow
  Boolean saturated;
  Medium.MassFraction X_sat = Medium.Xsaturation(state_evap) "Saturated water mass fraction for required evap temp";
  Medium.MassFraction X_sat_dry = Medium.xsaturation_pT(P_evap, T_evap) "Saturated water mass fraction w.r.t. to unit dry air mass for required evap state";
  Modelica.Units.SI.MassFlowRate w_cond "Flow rate of condensing water";
  Medium.AbsolutePressure P_evap "Pressure of air flow leaving evaporator";
  Medium.MassFraction X_evap(start=X_start_out) "Water mass ratio of air flow leaving evaporator";
  Medium.Temperature T_evap = PID_Temp.y "Temperature demanded of evaporator";
  Medium.ThermodynamicState state_evap "State of air flow leaving evaporator";
  Real H_rel_evap "Relative humidity of air flow leaving evaporator";
  // Inlet Flow
  Medium.ThermodynamicState state_inlet "State of inlet flow";
  Components.MassTransfer.SourceMassFlow outletMassFlow(
    use_in_massFlow=false,
    allowFlowReversal=false,
    use_in_T=true,
    use_in_Xw=false,
    use_di_massFlow=true,
    use_di_Xw=true,
    massFlow_di=m_fan,
    Xw_di=X_evap,
    use_di_T=false)
    annotation (Placement(transformation(extent={{-2,-60},{-22,-40}})));
  Components.MassTransfer.SourceMassFlow inletMassFlow(
    use_in_massFlow=false,
    allowFlowReversal=true,
    massFlow_nom=-m_fan,
    use_di_T=false,
    Xw_di=state_inlet.X[1],
    use_di_massFlow=true,
    massFlow_di=-m_fan,
    use_di_Xw=false)         annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={14,-50})));
  CustomInterfaces.FluidPort_A inletPortRecirc
    annotation (Placement(transformation(extent={{80,-20},{100,0}})));
  CustomInterfaces.FluidPort_B outletPort
    annotation (Placement(transformation(extent={{-100,-60},{-80,-40}})));
  Modelica.Blocks.Continuous.PID PID_Temp(
    y_start=T_start_out,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    xi_start=280,
    k=1,
    Ti=5,
    Td=0.1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-10,46})));
  Modelica.Blocks.Math.Add add(k1=-1, k2=+1)
    annotation (Placement(transformation(extent={{-46,64},{-26,84}})));
  Modelica.Blocks.Interfaces.RealInput plenumTemp
    annotation (Placement(transformation(extent={{-120,70},{-80,110}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a thermalPort
    annotation (Placement(transformation(extent={{-104,-10},{-84,10}})));
  Components.MassTransfer.Pipe duct(option=DynTherM.Choices.PDropOpt.fixed,
    m_flow_start=m_fan,
    T_start=T_start_out,
    X_start={X_start_out,1 - X_start_out})
    annotation (Placement(transformation(extent={{-54,-60},{-74,-40}})));
  Modelica.Blocks.Interfaces.RealInput targetTemp
    annotation (Placement(transformation(extent={{-120,30},{-80,70}})));
  CustomInterfaces.FluidPort_A inletPortFresh
    annotation (Placement(transformation(extent={{80,-64},{100,-44}})));
  Components.MassTransfer.simpleFan simpleFan(
    redeclare
      Components.MassTransfer.FanCharacteristics.FlowCharacteristics.linearFlow
      flowModel,
    volFlow_nom=0.5,
    Head_nom=100,
    eta_is=1,
    eta_m=nu_fan,
    omega_nom=600)
    annotation (Placement(transformation(extent={{-30,-40},{-50,-60}})));
  BoundaryConditions.mechanical mechanical(
    use_omega=true,
    use_in_omega=false,
    omega(displayUnit="rad/s") = 600)
    annotation (Placement(transformation(extent={{-9,-6},{9,6}},
        rotation=180,
        origin={-37,-82})));
equation
  P_evap = inletMassFlow.outlet.P - dP;
  state_inlet = Medium.setState_phX(inletMassFlow.outlet.P, actualStream(inletMassFlow.outlet.h_outflow), actualStream(inletMassFlow.outlet.Xi_outflow));
  state_evap = Medium.setState_pTX(P_evap, T_evap, {X_evap,1-X_evap});
  Q = m_fan*(Medium.specificEnthalpy(state_inlet) - Medium.specificEnthalpy(state_evap));
  Q_sens_cp = m_fan*Medium.specificHeatCapacityCp(state_inlet)*(state_inlet.T-T_evap) "Cp based sensible heat load";
  if X_sat <= state_inlet.X[1] then
    saturated = true;
    X_evap = X_sat;
    w_cond = m_fan*(state_inlet.X[1]-X_evap);
    Q_lat = w_cond*Medium.enthalpyOfVaporization(T_evap);
    Q_sens = Q - Q_lat;
    Q2 = Q_lat + Q_sens_cp;
    H_rel_evap = X_evap/X_sat;
  else
    saturated = false;
    X_evap = state_inlet.X[1];
    w_cond = 0;
    Q_lat = 0;
    Q2 = Q_lat + Q_sens_cp;
    Q_sens = m_fan*(Medium.enthalpyOfNonCondensingGas(state_inlet.T) - Medium.enthalpyOfNonCondensingGas(T_evap));
    H_rel_evap = X_evap/X_sat;
  end if;

  connect(add.y, PID_Temp.u)
    annotation (Line(points={{-25,74},{-10,74},{-10,58}}, color={0,0,127}));
  connect(PID_Temp.y, outletMassFlow.in_T)
    annotation (Line(points={{-10,35},{-10,-43}},         color={0,0,127}));
  connect(duct.outlet, outletPort) annotation (Line(points={{-74,-50},{-90,-50}},
                           color={0,0,0}));
  connect(duct.thermalPort, thermalPort)
    annotation (Line(points={{-64,-46.2},{-64,0},{-94,0}}, color={191,0,0}));
  connect(add.u1, plenumTemp)
    annotation (Line(points={{-48,80},{-100,80},{-100,90}}, color={0,0,127}));
  connect(targetTemp, add.u2) annotation (Line(points={{-100,50},{-82,50},{-82,
          48},{-48,48},{-48,68}}, color={0,0,127}));
  connect(duct.inlet, simpleFan.outlet)
    annotation (Line(points={{-54,-50},{-50,-50}}, color={0,0,0}));
  connect(simpleFan.inlet, outletMassFlow.outlet)
    annotation (Line(points={{-30,-50},{-22,-50}}, color={0,0,0}));
  connect(mechanical.mechanical, simpleFan.shaft)
    annotation (Line(points={{-40,-82},{-40,-60}}, color={135,135,135}));
  connect(inletPortRecirc, inletPortRecirc)
    annotation (Line(points={{90,-10},{90,-10}}, color={0,0,0}));
  connect(inletPortRecirc, inletMassFlow.outlet) annotation (Line(points={{90,-10},
          {48,-10},{48,-50},{24,-50}}, color={0,0,0}));
  connect(inletPortFresh, inletPortFresh)
    annotation (Line(points={{90,-54},{90,-54}}, color={0,0,0}));
  connect(inletPortFresh, inletMassFlow.outlet)
    annotation (Line(points={{90,-54},{24,-54},{24,-50}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-88,90},{84,-94}},
          lineColor={175,175,175},
          fillColor={0,0,0},
          fillPattern=FillPattern.Backward,
          lineThickness=0.5),
        Rectangle(
          extent={{-74,72},{70,62}},
          lineColor={175,175,175},
          lineThickness=0.5,
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-74,38},{70,28}},
          lineColor={175,175,175},
          lineThickness=0.5,
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-72,4},{72,-6}},
          lineColor={175,175,175},
          lineThickness=0.5,
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-70,-28},{74,-38}},
          lineColor={175,175,175},
          lineThickness=0.5,
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-72,-62},{72,-72}},
          lineColor={175,175,175},
          lineThickness=0.5,
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward)}),                   Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=200, __Dymola_NumberOfIntervals=1000));
end Evaporator;
