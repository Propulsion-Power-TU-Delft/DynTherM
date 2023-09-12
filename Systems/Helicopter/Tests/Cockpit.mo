within ThermalManagement.Systems.Helicopter.Tests;
model Cockpit "Testing the cockpit and evaporator together"
  package Medium = Modelica.Media.Air.MoistAir;
  parameter Medium.MassFraction X_env=0.01867;
  parameter Medium.MassFraction X_plenum=0.00894;
  parameter Medium.Temperature T_plenum = 28+273.15;
  parameter Real tau=0.96  "Recirculation ratio";
  Modelica.Blocks.Sources.Constant targetTempCabin(k=301.15)
    annotation (Placement(transformation(extent={{-74,-18},{-54,2}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=301.15)
    annotation (Placement(transformation(extent={{-86,18},{-66,38}})));

  Components.MassTransfer.SourceMassFlow sourceMassFlow(
    Xw_di=X_env,
    use_di_massFlow=true,
    use_di_T=true,
    use_di_Xw=true,
    massFlow_di=evaporator.m_fan*(1 - tau),
    allowFlowReversal=false,
    T_di=313.15)
    annotation (Placement(transformation(extent={{52,-8},{32,12}})));
  Subsystems.Evaporator evaporator(m_fan=0.3, T_start=280.32)
    annotation (Placement(transformation(extent={{-10,10},{10,30}})));
  NH90.BasicAirbusEES.AirbusCockpitDuct
                    airbusCockpit(N_occupants={20,0,2})
    annotation (Placement(transformation(extent={{-12,44},{8,64}})));
equation

  connect(evaporator.plenumTemp, realExpression.y) annotation (Line(points={{-10,29},
          {-20,29},{-20,28},{-65,28}},     color={0,0,127}));
  connect(targetTempCabin.y, evaporator.targetTemp) annotation (Line(points={{-53,-8},
          {-34,-8},{-34,25},{-10,25}},     color={0,0,127}));
  connect(sourceMassFlow.outlet, evaporator.inletPortFresh) annotation (Line(
        points={{32,2},{20,2},{20,14.6},{9,14.6}}, color={0,0,0}));
  connect(airbusCockpit.ductThermalPort, evaporator.thermalPort) annotation (
      Line(points={{7.2,49.8},{16,49.8},{16,36},{-22,36},{-22,20},{-9.4,20}},
        color={191,0,0}));
  connect(evaporator.outletPort, airbusCockpit.inletPort) annotation (Line(
        points={{-9,15},{-16,15},{-16,45.4},{-4.4,45.4}}, color={0,0,0}));
  connect(airbusCockpit.outletPort, evaporator.inletPortRecirc) annotation (
      Line(points={{0,45.4},{4,45.4},{4,44},{14,44},{14,19},{9,19}}, color={0,
          0,0}));
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
end Cockpit;
