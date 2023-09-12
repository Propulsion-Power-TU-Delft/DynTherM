within ThermalManagement.Systems.Helicopter.NH90.BasicAirbusEES;
model NH90Combined "Basic component built model (1:1 with EES)"
  parameter Modelica.Units.SI.MassFlowRate m_leak_in_ck=0.05
    "Cockpit leak in air flow" annotation (Dialog(group="Cockpit Airflow"));
  parameter Modelica.Units.SI.MassFlowRate m_fan_ck=0.3
    "Cockpit fan air through flow" annotation (Dialog(group="Cockpit Airflow"));
  parameter Modelica.Units.SI.MassFlowRate m_leak_in_cb=0.1
    "Cabin leak in air flow" annotation (Dialog(group="Cabin Airflow"));
  parameter Modelica.Units.SI.MassFlowRate m_fan_cb=0.6
    "Cabin fan air through flow" annotation (Dialog(group="Cabin Airflow"));
  parameter Modelica.Units.SI.MassFlowRate m_transfer=0.069
    "Transfer air from cabin and cockpit (>0)"
    annotation (Dialog(group="Cockpit Airflow"));
  parameter Real tau_ck=0.96 "Cockpit recirculation rate" annotation(Dialog(group="Cockpit Airflow"));
  parameter Real tau_cb=5/6 "Cabin recirculation rate" annotation(Dialog(group="Cabin Airflow"));
  parameter Real tau_leak = 0.5 "Leak out ratio cockpit/total";
  parameter Real N_occupants[3] = {20,0,2} "Number of occupants: passengers, crew, pilot";
  parameter Modelica.Units.SI.Temperature T_tgt[2]={301.15,301.15}
    "Target temperature: cockpit, plenum";
  final parameter Modelica.Units.SI.MassFlowRate m_ext_ck=m_fan_ck*(1 - tau_ck)
    "Cockpit external air into fan";
  final parameter Modelica.Units.SI.MassFlowRate m_ext_cb=m_fan_cb*(1 - tau_cb)
    "Cabin external air into fan";
  final parameter Modelica.Units.SI.MassFlowRate m_leak_out_ck=tau_leak*(
      m_leak_in_ck + m_leak_in_cb + m_ext_ck + m_ext_cb)
    "Cockpit leak out air flow";
  final parameter Modelica.Units.SI.MassFlowRate m_leak_out_cb=(1 - tau_leak)*(
      m_leak_in_ck + m_leak_in_cb + m_ext_ck + m_ext_cb) "Cabin leak out flow";
  Components.MassTransfer.SourceMassFlow leakInCabin(
    use_in_massFlow=false,
    use_in_T=false,
    use_in_Xw=false,
    allowFlowReversal=false,
    massFlow_nom=m_leak_in_cb,
    use_di_T=true,
    use_di_Xw=true,
    Xw_di=0.0187,
    T_di=313.15)                                         "air mass entering"
    annotation (Placement(transformation(extent={{-2,24},{18,44}})));
  Components.MassTransfer.SourceMassFlow leakOutCabin(
    allowFlowReversal=false,
    T_di=airbusCabin.plenum.T,
    Xw_di=airbusCabin.plenum.X[1],
    massFlow_nom=-m_leak_out_cb - 0.001,
    use_di_massFlow=false,
    use_di_T=true,
    use_di_Xw=true)
    annotation (Placement(transformation(extent={{92,34},{72,54}})));
  Components.MassTransfer.SourceMassFlow leakInCockpit(
    use_in_massFlow=false,
    use_in_T=false,
    use_in_Xw=false,
    allowFlowReversal=false,
    massFlow_nom=m_leak_in_ck,
    use_di_T=true,
    use_di_Xw=true,
    Xw_di=0.0187,
    T_di=313.15)
    annotation (Placement(transformation(extent={{-82,34},{-62,54}})));
  Subsystems.Evaporator evaporator_cb(m_fan=m_fan_cb)
    annotation (Placement(transformation(extent={{34,4},{54,24}})));
  Components.MassTransfer.SourceMassFlow airTransferCabin(
    use_in_massFlow=false,
    use_in_T=false,
    use_in_Xw=false,
    allowFlowReversal=false,
    massFlow_nom=-m_transfer,
    use_di_massFlow=false,
    use_di_T=true,
    use_di_Xw=true,
    T_di=airbusCabin.plenum.T,
    Xw_di=airbusCabin.plenum.X[1])
    annotation (Placement(transformation(extent={{92,16},{72,36}})));
  AirbusCockpitDuct airbusCockpit(N_occupants={0,0,N_occupants[3]}, T_start=
        T_tgt[1],
    W_vitre_ck=0)
    annotation (Placement(transformation(extent={{-48,42},{-28,62}})));
  AirbusCabinDuct airbusCabin(N_occupants={N_occupants[1],0,0},
    T_start=T_tgt[2],
    T_engine=373.15,
    T_btp=353.15)
    annotation (Placement(transformation(extent={{32,30},{52,50}})));
  Components.MassTransfer.SourceMassFlow leakOutCockpit(
    use_in_massFlow=false,
    use_in_T=false,
    use_in_Xw=false,
    allowFlowReversal=false,
    T_di=airbusCockpit.plenum.T,
    Xw_di=airbusCockpit.plenum.X[1],
    massFlow_nom=-m_leak_out_ck - 0.0001,
    use_di_T=true,
    use_di_Xw=true)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-20,74})));
  Components.MassTransfer.SourceMassFlow airTransferCockpit(
    use_in_massFlow=false,
    use_in_T=false,
    use_in_Xw=false,
    allowFlowReversal=false,
    massFlow_nom=m_transfer,
    use_di_T=true,
    use_di_Xw=true,
    T_di=airTransferCabin.T_di,
    Xw_di=airTransferCabin.Xw_di)
    annotation (Placement(transformation(extent={{-82,18},{-62,38}})));
  Modelica.Blocks.Sources.Constant targetTempCabin(k=T_tgt[2])
    annotation (Placement(transformation(extent={{-2,-78},{18,-58}})));
  Modelica.Blocks.Sources.Constant targetTempCockpit(k=T_tgt[1])
    annotation (Placement(transformation(extent={{-82,-78},{-62,-58}})));
  Subsystems.Evaporator evaporator_ck(m_fan=m_fan_ck)
    annotation (Placement(transformation(extent={{-46,10},{-26,30}})));
  Components.MassTransfer.SourceMassFlow extAirCb(
    allowFlowReversal=false,
    use_di_T=true,
    use_di_Xw=true,
    Xw_di=0.0187,
    massFlow_nom=m_fan_cb,
    use_di_massFlow=true,
    massFlow_di=m_ext_cb,
    T_di=313.15)
    annotation (Placement(transformation(extent={{92,0},{72,20}})));
  Components.MassTransfer.SourceMassFlow extAirCk(
    allowFlowReversal=false,
    use_di_T=true,
    use_di_Xw=true,
    Xw_di=0.0187,
    massFlow_nom=m_fan_ck,
    use_di_massFlow=true,
    massFlow_di=m_ext_ck,
    T_di=313.15)
    annotation (Placement(transformation(extent={{6,4},{-14,24}})));
  inner Components.Environment environment(
    extSw=true,
    ISA_plus=25,
    allowFlowReversal=false,
    initOpt=ThermalManagement.Choices.InitOpt.fixedState,
    phi_amb=0.4,
    T_ground=273.15)
    annotation (Placement(transformation(extent={{-88,72},{-68,92}})));
equation

  connect(leakInCockpit.outlet, airbusCockpit.inletPort) annotation (Line(
        points={{-62,44},{-58,44},{-58,43.4},{-40.4,43.4}}, color={0,0,0}));
  connect(airTransferCockpit.outlet, airbusCockpit.inletPort) annotation (Line(
        points={{-62,28},{-56,28},{-56,43.4},{-40.4,43.4}}, color={0,0,0}));
  connect(evaporator_ck.plenumTemp, airbusCockpit.plenumTemp) annotation (Line(
        points={{-46,29},{-52,29},{-52,60},{-49,60}}, color={0,0,127}));
  connect(evaporator_ck.targetTemp, targetTempCockpit.y) annotation (Line(
        points={{-46,25},{-52,25},{-52,-68},{-61,-68}}, color={0,0,127}));
  connect(airbusCabin.outletPort,evaporator_cb. inletPortRecirc) annotation (
      Line(points={{44.4,31.2},{62,31.2},{62,13},{53,13}}, color={0,0,0}));
  connect(leakOutCabin.outlet,evaporator_cb. inletPortRecirc)
    annotation (Line(points={{72,44},{62,44},{62,13},{53,13}}, color={0,0,0}));
  connect(airTransferCabin.outlet,evaporator_cb. inletPortRecirc)
    annotation (Line(points={{72,26},{62,26},{62,13},{53,13}}, color={0,0,0}));
  connect(evaporator_cb.plenumTemp, airbusCabin.plenumTemp) annotation (Line(
        points={{34,23},{28,23},{28,48},{31,48}}, color={0,0,127}));
  connect(evaporator_ck.outletPort, airbusCockpit.inletPort) annotation (Line(
        points={{-45,15},{-56,15},{-56,43.4},{-40.4,43.4}}, color={0,0,0}));
  connect(evaporator_ck.inletPortRecirc, leakOutCockpit.outlet)
    annotation (Line(points={{-27,19},{-20,19},{-20,64}}, color={0,0,0}));
  connect(airbusCockpit.outletPort, leakOutCockpit.outlet) annotation (Line(
        points={{-36,43.4},{-36,44},{-20,44},{-20,64}}, color={0,0,0}));
  connect(extAirCb.outlet,evaporator_cb. inletPortFresh)
    annotation (Line(points={{72,10},{62,10},{62,8.6},{53,8.6}},
                                                               color={0,0,0}));
  connect(extAirCk.outlet,evaporator_ck. inletPortFresh) annotation (Line(
        points={{-14,14},{-14,14.6},{-27,14.6}},        color={0,0,0}));
  connect(evaporator_cb.targetTemp, targetTempCabin.y) annotation (Line(points={
          {34,19},{32,19},{32,18},{28,18},{28,-68},{19,-68}}, color={0,0,127}));
  connect(airbusCabin.inletPort, evaporator_cb.outletPort) annotation (Line(
        points={{39.4,31.2},{24,31.2},{24,9},{35,9}}, color={0,0,0}));
  connect(leakInCabin.outlet, evaporator_cb.outletPort)
    annotation (Line(points={{18,34},{24,34},{24,9},{35,9}}, color={0,0,0}));
  connect(evaporator_ck.thermalPort, airbusCockpit.ductThermalPort) annotation (
     Line(points={{-45.4,20},{-60,20},{-60,36},{-24,36},{-24,47.8},{-28.8,47.8}},
        color={191,0,0}));
  connect(evaporator_cb.thermalPort, airbusCabin.ductThermalPort) annotation (
      Line(points={{34.6,14},{20,14},{20,26},{58,26},{58,35.8},{51.4,35.8}},
        color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=500, __Dymola_NumberOfIntervals=1000));
end NH90Combined;
