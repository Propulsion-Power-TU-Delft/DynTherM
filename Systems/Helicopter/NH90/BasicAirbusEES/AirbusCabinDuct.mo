within ThermalManagement.Systems.Helicopter.NH90.BasicAirbusEES;
model AirbusCabinDuct
  outer Components.Environment environment;
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
  parameter Modelica.Units.SI.Temperature T_engine "Engine temperature"
    annotation (Dialog(group="Temperature"));
  parameter Modelica.Units.SI.Temperature T_btp "Transmission temperature"
    annotation (Dialog(group="Temperature"));
  parameter Real N_occupants[3] "Number of: passengers, cabin crew, pilots inside the cabin" annotation(Dialog(group="Inputs"));
  parameter Modelica.Units.SI.MassFlowRate m_H2O=0.00005
    "Water vapour per occupant" annotation (Dialog(group="Inputs"));
  parameter Modelica.Units.SI.HeatFlowRate Q_occupants=130
    "Heat input per occupant" annotation (Dialog(group="Inputs"));
  parameter Modelica.Units.SI.HeatFlowRate Q_avionics=750
    "Heat input from avionics" annotation (Dialog(group="Inputs"));
  parameter Modelica.Units.SI.MassFraction X_start[2]={0.0124,0.9876}
    "Start gas composition" annotation (Dialog(tab="Initialization"));
  // Duct
  Components.HeatTransfer.InternalConvection ductIntConvection(A=A_duct,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_fixed))
    annotation (Placement(transformation(extent={{44,-22},{64,-2}})));
  Components.HeatTransfer.WallConduction ductConduction(
    redeclare model Mat = Materials.AirbusEES.Duct,
    t=t_duct,
    A=A_duct,
    Tstart=T_start,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{44,-34},{64,-14}})));
  Components.HeatTransfer.InternalConvection ductExtConduction(A=A_duct,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_duct_fixed))
    annotation (Placement(transformation(extent={{44,-46},{64,-26}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a ductThermalPort
    annotation (Placement(transformation(extent={{84,-52},{104,-32}})));
  // Floor
  Components.HeatTransfer.WallConduction floorConduction(
    t=t_floor,
    A=A_floor,
    redeclare model Mat = Materials.AirbusEES.Floor,
    Tstart=T_start,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{44,20},{64,40}})));
  Components.HeatTransfer.InternalConvection floorIntConvection(A=A_floor,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_fixed))
    annotation (Placement(transformation(extent={{44,32},{64,52}})));
  Components.HeatTransfer.InternalConvection floorExtConvection(A=A_floor,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_floor_fixed))
    annotation (Placement(transformation(extent={{44,8},{64,28}})));
  BoundaryConditions.thermal floorExtTempInput(use_T=true, T=T_floor)
    annotation (Placement(transformation(extent={{50,8},{56,12}})));
  // Fuselage
  Components.HeatTransfer.InternalConvection fusIntConvection(A=A_fus,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_fixed))                               annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-38,4})));
  Components.HeatTransfer.WallConduction fusConduction(
    t=t_fus,
    A=A_fus,
    redeclare model Mat = Materials.AirbusEES.Fuselage,
    Tstart=T_start,
    initOpt=environment.initOpt)
                   annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-52,4})));
  Components.MassTransfer.Plenum plenum(
    noInitialPressure=false,
    noInitialTemperature=false,
    initOpt=ThermalManagement.Choices.InitOpt.fixedState,
    Q_int=Q_avionics,
    Q_sens_fixed=N_occupants[1]*Q_occupants,
    m_H2O_fixed=N_occupants[1]*m_H2O,
    N_occupants=N_occupants,
    allowFlowReversal=false,
    X_start=X_start,
    fixed_Q_sens=true,
    V=A_floor*A_fus/2/LX)
    annotation (Placement(transformation(extent={{-6,-86},{14,-66}})));
  Components.HeatTransfer.ExternalConvection fusExtConvection(A=A_fus,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue (
          ht_fixed=ht_ext_fixed))                             annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-66,4})));
  // Window
  BoundaryConditions.thermal windowHeatInput(use_Q=true, Q=-E_sun*Ap_window*
        R_window)                                                               annotation (Placement(transformation(extent={{-46,54},{-38,50}})));
  Components.HeatTransfer.InternalConvection windowIntConvection(A=A_window,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_fixed))                                 annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-38,38})));
  Components.HeatTransfer.WallConduction windowConduction(
    redeclare model Mat = Materials.AirbusEES.Window,
    Tstart=T_start,
    initOpt=environment.initOpt,
    t=t_window,
    A=A_window)
             annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-52,38})));
  Components.HeatTransfer.ExternalConvection windowExtConvection(A=A_window,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue (
          ht_fixed=ht_ext_fixed))
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-68,38})));
  // Ports
  CustomInterfaces.FluidPort_A inletPort
    annotation (Placement(transformation(extent={{-36,-98},{-16,-78}})));
  CustomInterfaces.FluidPort_B outletPort
    annotation (Placement(transformation(extent={{14,-98},{34,-78}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-22,-66},{-42,-46}})));
  Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{52,-86},{72,-66}})));
  Modelica.Blocks.Interfaces.RealOutput plenumTemp
    annotation (Placement(transformation(extent={{-100,70},{-120,90}})));
  Modelica.Blocks.Interfaces.RealOutput plenumPressure
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  Components.HeatTransfer.InternalConvection engineIntConvection(A=A_engine,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_fixed))  annotation (Placement(transformation(extent={{-22,54},
            {-2,74}})));
  Components.HeatTransfer.WallConduction engineConduction(
    redeclare model Mat = Materials.AirbusEES.EngineAluminium,
    A=A_engine,
    t=t_engine,
    Tstart=T_start,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-22,66},{-2,86}})));
  BoundaryConditions.thermal engineTemp(
    T=T_engine,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-18,90},{-8,98}})));
  Components.HeatTransfer.InternalConvection engineExtConvection(A=A_engine,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_engine_fixed))      annotation (Placement(transformation(extent={{-22,76},
            {-2,96}})));
  Components.HeatTransfer.InternalConvection transmissionExtConvection(A=A_btp,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_btp_fixed))      annotation (Placement(transformation(extent={{24,76},
            {44,96}})));
  Components.HeatTransfer.WallConduction TransmissionWall(
    redeclare model Mat = Materials.AirbusEES.TransmissionAluminium,
    A=A_btp,
    t=t_btp,
    Tstart=T_start,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{24,64},{44,84}})));
  BoundaryConditions.thermal transmissionTemp(
    T=T_btp,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{26,92},{38,100}})));
  Components.HeatTransfer.InternalConvection transmissionIntConvection(
                                     A=A_btp, redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_fixed))             annotation (Placement(transformation(extent={{24,52},
            {44,72}})));
equation
  connect(floorIntConvection.outlet, floorConduction.inlet)
    annotation (Line(points={{54,38.6},{54,33.4}},   color={191,0,0}));
  connect(fusConduction.inlet, fusIntConvection.outlet)
    annotation (Line(points={{-48.6,4},{-41.4,4}}, color={191,0,0}));
  connect(floorExtConvection.inlet, floorConduction.outlet)
    annotation (Line(points={{54,21.4},{54,26.6}},   color={191,0,0}));
  connect(floorExtTempInput.thermal, floorExtConvection.outlet)
    annotation (Line(points={{54,10},{54,14.6}}, color={191,0,0}));
  connect(outletPort, outletPort)
    annotation (Line(points={{24,-88},{24,-88}},          color={0,0,0}));
  connect(outletPort, plenum.outlet)
    annotation (Line(points={{24,-88},{24,-76},{14,-76}}, color={0,0,0}));
  connect(inletPort, plenum.inlet)
    annotation (Line(points={{-26,-88},{-26,-76},{-6,-76}},color={0,0,0}));
  connect(pressureSensor.port, plenum.outlet)
    annotation (Line(points={{52,-76},{14,-76}}, color={0,0,0}));
  connect(pressureSensor.y, plenumPressure)
    annotation (Line(points={{73,-76},{80,-76},{80,80},{110,80}},
                                                 color={0,0,127}));
  connect(inletPort, inletPort) annotation (Line(points={{-26,-88},{-26,-88}},
                                color={0,0,0}));
  connect(temperatureSensor.T, plenumTemp)
    annotation (Line(points={{-43,-56},{-80,-56},{-80,80},{-110,80}},
                                                   color={0,0,127}));
  connect(ductConduction.inlet, ductIntConvection.outlet)
    annotation (Line(points={{54,-20.6},{54,-15.4}}, color={191,0,0}));
  connect(ductExtConduction.inlet, ductConduction.outlet)
    annotation (Line(points={{54,-32.6},{54,-27.4}}, color={191,0,0}));
  connect(ductThermalPort, ductExtConduction.outlet) annotation (Line(
        points={{94,-42},{94,-39.4},{54,-39.4}}, color={191,0,0}));
  connect(ductThermalPort, ductThermalPort) annotation (Line(points={{94,
          -42},{94,-42},{94,-42}}, color={191,0,0}));
  connect(windowConduction.inlet,windowIntConvection. outlet)
    annotation (Line(points={{-48.6,38},{-41.4,38}}, color={191,0,0}));
  connect(plenum.thermalPort, temperatureSensor.port)
    annotation (Line(points={{4,-67},{4,-56},{-22,-56}}, color={191,0,0}));
  connect(ductIntConvection.inlet, temperatureSensor.port) annotation (Line(
        points={{54,-8.6},{54,0},{4,0},{4,-56},{-22,-56}},    color={191,0,
          0}));
  connect(floorIntConvection.inlet, temperatureSensor.port) annotation (
      Line(points={{54,45.4},{54,54},{4,54},{4,-56},{-22,-56}}, color={191,
          0,0}));
  connect(windowIntConvection.inlet, temperatureSensor.port) annotation (
      Line(points={{-34.6,38},{4,38},{4,-56},{-22,-56}}, color={191,0,0}));
  connect(windowHeatInput.thermal, temperatureSensor.port) annotation (Line(
        points={{-40.6667,52},{4,52},{4,-56},{-22,-56}}, color={191,0,0}));
  connect(fusIntConvection.inlet, temperatureSensor.port) annotation (Line(
        points={{-34.6,4},{4,4},{4,-56},{-22,-56}}, color={191,0,0}));
  connect(windowExtConvection.inlet, windowConduction.outlet)
    annotation (Line(points={{-64.6,38},{-55.4,38}}, color={191,0,0}));
  connect(fusExtConvection.inlet, fusConduction.outlet)
    annotation (Line(points={{-62.6,4},{-55.4,4}}, color={191,0,0}));
  connect(TransmissionWall.outlet,transmissionIntConvection. inlet)
    annotation (Line(points={{34,70.6},{34,65.4}}, color={191,0,0}));
  connect(engineConduction.outlet,engineIntConvection. inlet)
    annotation (Line(points={{-12,72.6},{-12,67.4}}, color={191,0,0}));
  connect(transmissionExtConvection.outlet,TransmissionWall. inlet)
    annotation (Line(points={{34,82.6},{34,77.4}}, color={191,0,0}));
  connect(transmissionExtConvection.inlet,transmissionTemp. thermal)
    annotation (Line(points={{34,89.4},{36,89.4},{36,96},{34,96}}, color={191,0,
          0}));
  connect(engineExtConvection.outlet,engineConduction. inlet)
    annotation (Line(points={{-12,82.6},{-12,79.4}}, color={191,0,0}));
  connect(engineTemp.thermal,engineExtConvection. inlet) annotation (Line(
        points={{-11.3333,94},{-12,94},{-12,89.4}}, color={191,0,0}));
  connect(transmissionIntConvection.outlet, temperatureSensor.port) annotation (
     Line(points={{34,58.6},{4,58.6},{4,-56},{-22,-56}}, color={191,0,0}));
  connect(engineIntConvection.outlet, temperatureSensor.port) annotation (Line(
        points={{-12,60.6},{4,60.6},{4,-56},{-22,-56}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{34,48},{74,8}},    lineColor={28,108,200}),
        Text(
          extent={{34,52},{56,48}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Floor"),
        Rectangle(extent={{-74,20},{-26,-10}}, lineColor={28,108,200}),
        Text(
          extent={{-76,26},{-56,20}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Fuselage"),
        Rectangle(extent={{32,-6},{72,-46}},  lineColor={28,108,200}),
        Text(
          extent={{28,-2},{50,-6}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Duct"),
        Rectangle(extent={{-74,54},{-26,26}}, lineColor={28,108,200}),
        Text(
          extent={{-76,60},{-60,54}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Window")}));
end AirbusCabinDuct;
