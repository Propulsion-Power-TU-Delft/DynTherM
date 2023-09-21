within DynTherM.Systems.Helicopter.Tests;
model Cabin "Testing the cabin and evaporator together"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium                                                                      "Medium model" annotation(choicesAllMatching = true);
  parameter Medium.MassFraction X_env=0.01867;
  parameter Real tau=5/6 "Recirculation ratio";

  Modelica.Blocks.Sources.Constant targetTempCabin(k=301.15)
    annotation (Placement(transformation(extent={{-52,14},{-32,34}})));

  BoundaryConditions.flow_source sourceMassFlow(
    Xw_di=X_env,
    use_di_massFlow=true,
    use_di_T=true,
    use_di_Xw=true,
    massFlow_di=evaporator.m_fan*(1 - tau),
    allowFlowReversal=false,
    T_di=313.15)
    annotation (Placement(transformation(extent={{52,4},{32,24}})));
  Subsystems.Evaporator evaporator(
    m_fan=0.6,
    T_start_out=285.98,
    X_start_out=0.00922)
    annotation (Placement(transformation(extent={{-10,10},{10,30}})));
  NH90.BasicAirbusEES.AirbusCabinDuct airbusCabin(
    T_engine=353.15,
    T_btp=353.15,
    N_occupants={20,0,2},
    X_start={0.0124,0.9876})
    annotation (Placement(transformation(extent={{-12,48},{8,68}})));
  inner Components.Environment environment(ISA_plus=25, phi_amb=0.4)
    annotation (Placement(transformation(extent={{-86,50},{-66,70}})));
equation

  connect(targetTempCabin.y, evaporator.targetTemp) annotation (Line(points={{-31,24},
          {-28,24},{-28,25},{-10,25}},     color={0,0,127}));
  connect(sourceMassFlow.outlet, evaporator.inletPortFresh) annotation (Line(
        points={{32,14},{30,14},{30,14.6},{9,14.6}},
                                                   color={0,0,0}));
  connect(evaporator.thermalPort, airbusCabin.ductThermalPort) annotation (
      Line(points={{-9.4,20},{-14,20},{-14,44},{16,44},{16,53.8},{7.4,53.8}},
        color={191,0,0}));
  connect(evaporator.outletPort, airbusCabin.inletPort) annotation (Line(
        points={{-9,15},{-18,15},{-18,49.2},{-4.6,49.2}}, color={0,0,0}));
  connect(airbusCabin.plenumTemp, evaporator.plenumTemp) annotation (Line(
        points={{-13,66},{-24,66},{-24,29},{-10,29}}, color={0,0,127}));
  connect(airbusCabin.outletPort, evaporator.inletPortRecirc) annotation (
      Line(points={{0.4,49.2},{0.4,50},{18,50},{18,19},{9,19}}, color={0,0,0}));
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
end Cabin;
