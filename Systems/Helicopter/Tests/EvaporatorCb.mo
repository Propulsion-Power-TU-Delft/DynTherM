within DynTherM.Systems.Helicopter.Tests;
model EvaporatorCb
  "Testing the evaporator solely with EES cabin inputs"
  package Medium = Modelica.Media.Air.MoistAir;
  parameter Medium.MassFraction X_env=0.01867;
  parameter Medium.MassFraction X_plenum=0.012;
  parameter Medium.Temperature T_plenum=301.15;
  parameter Real tau=5/6  "Recirculation ratio";
  parameter Modelica.Units.SI.HeatFlowRate W_duct = -169;

  BoundaryConditions.thermal cockpitIntoDuct(use_Q=true, Q=W_duct)
    annotation (Placement(transformation(extent={{-28,16},{-22,22}})));
  Modelica.Blocks.Sources.Constant targetTempCabin(k=301.15)
    annotation (Placement(transformation(extent={{-62,-28},{-42,-8}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=301.15)
    annotation (Placement(transformation(extent={{-64,18},{-44,38}})));

  Components.MassTransfer.SourceMassFlow sourceMassFlow(
    Xw_di=X_env,
    use_di_massFlow=true,
    use_di_T=true,
    use_di_Xw=true,
    massFlow_di=evaporator.m_fan*(1 - tau),
    allowFlowReversal=false,
    T_di=313.15)
    annotation (Placement(transformation(extent={{52,-8},{32,12}})));
  Subsystems.Evaporator evaporator(
    m_fan=0.6,
    T_start_out=285.98,
    X_start_out=0.00922,
    nu_fan=0.9)
    annotation (Placement(transformation(extent={{-10,10},{10,30}})));
  Components.MassTransfer.PressureSink plenumOut(
    use_ambient=false,
    T_di=T_plenum,
    Xw_di=X_plenum)
    annotation (Placement(transformation(extent={{32,10},{52,30}})));
  Components.MassTransfer.PressureSink plenumIn(
    use_ambient=false,
    T_di=T_plenum,
    Xw_di=X_plenum)
    annotation (Placement(transformation(extent={{-64,-2},{-84,18}})));
equation

  connect(evaporator.thermalPort, cockpitIntoDuct.thermal) annotation (Line(
        points={{-9.4,20},{-24,20},{-24,19}},                   color={191,0,0}));
  connect(evaporator.plenumTemp, realExpression.y) annotation (Line(points={{-10,29},
          {-10,28},{-43,28}},              color={0,0,127}));
  connect(sourceMassFlow.outlet, evaporator.inletPortFresh) annotation (Line(
        points={{32,2},{20,2},{20,14.6},{9,14.6}}, color={0,0,0}));
  connect(plenumOut.inlet, evaporator.inletPortRecirc) annotation (Line(points={
          {32,20},{20.5,20},{20.5,19},{9,19}}, color={0,0,0}));
  connect(plenumIn.inlet, evaporator.outletPort) annotation (Line(points={{-64,8},
          {-16,8},{-16,15},{-9,15}}, color={0,0,0}));
  connect(targetTempCabin.y, evaporator.targetTemp) annotation (Line(points={{-41,
          -18},{-34,-18},{-34,25},{-10,25}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Text(
          extent={{-46,90},{0,80}},
          lineColor={28,108,200},
          textString="Q: "+DynamicSelect("0", String(evaporator.Q2))+
          "\nQ_lat: "+DynamicSelect("0", String(evaporator.Q_lat))+
          "\nQ_sens: "+DynamicSelect("0", String(evaporator.Q_sens2)))}),
    experiment(
      StopTime=500,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end EvaporatorCb;
