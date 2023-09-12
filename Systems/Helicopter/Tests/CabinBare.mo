within DynTherM.Systems.Helicopter.Tests;
model CabinBare
  "Testing the cabin bare elements and evaporator together"
  package Medium = Modelica.Media.Air.MoistAir;
  parameter Medium.MassFraction X_env=0.01867;
  parameter Medium.MassFraction X_plenum=0.00894;
  parameter Medium.Temperature T_plenum = 28+273.15;

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
  parameter Modelica.Units.SI.Temperature T_start=300 "Start temperature";
  parameter Modelica.Units.SI.Length LX=6
    "Fuselage X-length (Arbitrary - used for units)"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_fus=22 "Fuselage wall surface area"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_fus=0.03 "Fuselage wall thickness"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_floor=12.6 "Floor surface area"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_floor=0.025 "Floor thickness"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_duct=2 "Duct surface area"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_duct=0.005 "Duct thickness"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_window=5 "Window area"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area Ap_window=0.25 "Window projected area"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_window=0.01 "Window wall thickness"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length LX_window=2 "Window length"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_engine=4.2 "Engine wall surface area"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_engine=0.04 "Engine wall thickness"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_btp=4.2 "Transmission wall surface area"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_btp=0.04 "Exhaust wall thickness"
    annotation (Dialog(group="Dimensions"));

  parameter Real R_window = 0.92 "Window transmissivity coefficient";
  parameter Modelica.Units.SI.Irradiance E_sun=900;
  parameter Modelica.Units.SI.Length L_duct=2 "Duct length"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length W_duct=1 "Duct width"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_int_fixed=15
    "Internal convection fixed heat transfer coefficient";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_ext_fixed=100
    "External convection fixed heat transfer coefficient";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_floor_fixed=25
    "Floor external convection fixed heat transfer coefficient";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_duct_fixed=80
    "Duct internal convection fixed heat transfer coefficient";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_btp_fixed=25
    "Transmission convection fixed heat transfer coefficient";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_engine_fixed=25
    "Motor convection fixed heat transfer coefficient";
  parameter Modelica.Units.SI.Temperature T_floor=323.15 "Floor temperature"
    annotation (Dialog(group="Temperature"));
  parameter Modelica.Units.SI.Temperature T_engine=353.15 "Engine temperature"
    annotation (Dialog(group="Temperature"));
  parameter Modelica.Units.SI.Temperature T_btp=353.15 "Transmission temperature"
    annotation (Dialog(group="Temperature"));
  parameter Modelica.Units.SI.MassFlowRate m_H2O=0.00005
    "Water vapour per occupant" annotation (Dialog(group="Inputs"));
  parameter Modelica.Units.SI.HeatFlowRate Q_occupants=130
    "Heat input per occupant" annotation (Dialog(group="Inputs"));
  parameter Modelica.Units.SI.HeatFlowRate Q_avionics=750
    "Heat input from avionics" annotation (Dialog(group="Inputs"));
  parameter Modelica.Units.SI.MassFraction X_start[2]={0.0124,0.9876}
    "Start gas composition" annotation (Dialog(tab="Initialization"));
  Modelica.Blocks.Sources.Constant targetTempCabin(k=301.15)
    annotation (Placement(transformation(extent={{-66,-92},{-46,-72}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=301.15)
    annotation (Placement(transformation(extent={{-78,-56},{-58,-36}})));

  Components.MassTransfer.SourceMassFlow sourceMassFlow(
    Xw_di=X_env,
    use_di_massFlow=true,
    use_di_T=true,
    use_di_Xw=true,
    massFlow_di=evaporator.m_fan*(1 - tau_cb),
    allowFlowReversal=false,
    T_di=313.15)
    annotation (Placement(transformation(extent={{60,-82},{40,-62}})));
  Subsystems.Evaporator evaporator(m_fan=0.3, T_start_out=553.47)
    annotation (Placement(transformation(extent={{-2,-64},{18,-44}})));
  Components.MassTransfer.Plenum plenum(
    noInitialPressure=false,
    noInitialTemperature=false,
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    Q_int=Q_avionics,
    Q_sens_fixed=N_occupants[1]*Q_occupants,
    m_H2O_fixed=N_occupants[1]*m_H2O,
    N_occupants=N_occupants,
    allowFlowReversal=false,
    X_start=X_start,
    fixed_Q_sens=true,
    V=A_floor*A_fus/2/LX)
    annotation (Placement(transformation(extent={{-6,-8},{14,12}})));
  inner Components.Environment environment
    annotation (Placement(transformation(extent={{46,70},{66,90}})));
  BoundaryConditions.thermal duct(use_Q=true, Q=-200)
    annotation (Placement(transformation(extent={{-34,-32},{-26,-36}})));
  BoundaryConditions.thermal plenumheat(use_Q=true, Q=-2000)
    annotation (Placement(transformation(extent={{-2,22},{4,18}})));
  Components.MassTransfer.SourceMassFlow leakInCabin(
    use_in_massFlow=false,
    use_in_T=false,
    use_in_Xw=false,
    allowFlowReversal=false,
    massFlow_nom=m_leak_in_cb,
    use_di_massFlow=true,
    use_di_T=true,
    use_di_Xw=true,
    massFlow_di=m_leak_in_cb,
    Xw_di=0.0187,
    T_di=313.15)                                         "air mass entering"
    annotation (Placement(transformation(extent={{-64,-8},{-44,12}})));
  Components.MassTransfer.SourceMassFlow airTransferCabin(
    use_in_massFlow=false,
    use_in_T=false,
    use_in_Xw=false,
    allowFlowReversal=true,
    use_di_massFlow=true,
    use_di_T=true,
    use_di_Xw=true,
    massFlow_di=-m_transfer,
    T_di=plenum.T,
    Xw_di=plenum.X[1])
    annotation (Placement(transformation(extent={{52,-50},{32,-30}})));
equation

  connect(evaporator.plenumTemp, realExpression.y) annotation (Line(points={{-2,-45},
          {-12,-45},{-12,-46},{-57,-46}},  color={0,0,127}));
  connect(targetTempCabin.y, evaporator.targetTemp) annotation (Line(points={{-45,-82},
          {-26,-82},{-26,-49},{-2,-49}},   color={0,0,127}));
  connect(sourceMassFlow.outlet, evaporator.inletPortFresh) annotation (Line(
        points={{40,-72},{28,-72},{28,-59.4},{17,-59.4}},
                                                   color={0,0,0}));
  connect(plenum.outlet, evaporator.inletPortRecirc)
    annotation (Line(points={{14,2},{24,2},{24,-55},{17,-55}}, color={0,0,0}));
  connect(plenum.inlet, evaporator.outletPort) annotation (Line(points={{-6,2},{
          -14,2},{-14,-59},{-1,-59}}, color={0,0,0}));
  connect(duct.thermal, evaporator.thermalPort) annotation (Line(points={{
          -28.6667,-34},{-30,-34},{-30,-54},{-1.4,-54}},
                                                color={191,0,0}));
  connect(plenumheat.thermal, plenum.thermalPort)
    annotation (Line(points={{2,20},{2,11},{4,11}},       color={191,0,0}));
  connect(leakInCabin.outlet, evaporator.outletPort) annotation (Line(points=
          {{-44,2},{-14,2},{-14,-59},{-1,-59}}, color={0,0,0}));
  connect(airTransferCabin.outlet, evaporator.inletPortRecirc) annotation (
      Line(points={{32,-40},{24,-40},{24,-55},{17,-55}}, color={0,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Text(
          extent={{-112,92},{-66,82}},
          lineColor={28,108,200},
          textString="Q: "+DynamicSelect("0", String(evaporator.Q2))+
          "\nQ_lat: "+DynamicSelect("0", String(evaporator.Q_lat))+
          "\nQ_sens: "+DynamicSelect("0", String(evaporator.Q_sens2)))}),
    experiment(
      StopTime=100,
      __Dymola_NumberOfIntervals=500,
      __Dymola_Algorithm="Dassl"));
end CabinBare;
