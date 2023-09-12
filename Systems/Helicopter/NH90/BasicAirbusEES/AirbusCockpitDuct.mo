within DynTherM.Systems.Helicopter.NH90.BasicAirbusEES;
model AirbusCockpitDuct
  outer Components.Environment environment;
  parameter Modelica.Units.SI.Temperature T_start=300 "Start temperature";
  parameter Modelica.Units.SI.Length LX=1
    "Fuselage X-length (Arbitrary - used for units)" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_fus=2.5 "Fuselage wall surface area" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_fus=0.03 "Fuselage wall thickness" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_floor=2.5 "Floor surface area" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_floor=0.025 "Floor thickness" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_duct=2 "Duct surface area" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_duct=0.0001 "Duct thickness" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_win=4.3 "Window area" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area Ap_win=3.5 "Window projected area" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_win=0.01 "Window thickness" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length LX_win=1 "Window length" annotation (Dialog(group="Dimensions"));
  parameter Real R_win = 0.98 "Window transmissivity coefficient";
  parameter Modelica.Units.SI.HeatFlowRate W_vitre_ck=-433 "Window heat input";
  parameter Modelica.Units.SI.Irradiance E_sun=900;
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_int_fixed=15
    "Internal convection fixed heat transfer coefficient";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_ext_fixed=80
    "External convection fixed heat transfer coefficient";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_floor_fixed=25
    "Floor external convection fixed heat transfer coefficient";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_int_duct=80
    "Duct internal convection fixed heat transfer coefficient";
  parameter Modelica.Units.SI.Temperature T_floor=323.15 "Floor temperature";
  parameter Real N_occupants[3]
    "Number of: passengers, cabin crew, pilots inside the cabin" annotation(Dialog(group="Inputs"));
  parameter Modelica.Units.SI.MassFlowRate m_H2O=0.00005
    "Water vapour per occupant" annotation (Dialog(group="Inputs"));
  parameter Modelica.Units.SI.HeatFlowRate Q_occupants=130
    "Heat input per occupant" annotation (Dialog(group="Inputs"));
  parameter Modelica.Units.SI.HeatFlowRate Q_avionics=1000
    "Heat input from avionics" annotation (Dialog(group="Inputs"));
  parameter Modelica.Units.SI.MassFraction X_start[2]={0.01,0.99}
    "Start gas composition" annotation (Dialog(tab="Initialization"));
  // Windscreen
//   Modelica.SIunits.Temperature T_w_int "Glass internal temperature";
//   Modelica.SIunits.Temperature T_w_ext "Glass external temperature";
//   Modelica.SIunits.HeatFlowRate W_ext "Glass external heat flow rate";
//   Modelica.SIunits.HeatFlowRate W_int "Glass internal heat flow rate";
  // Duct
  Components.HeatTransfer.InternalConvection ductIntConvection(A=A_duct,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_fixed))
    annotation (Placement(transformation(extent={{48,-24},{68,-4}})));
  Components.HeatTransfer.WallConduction ductConduction(
    redeclare model Mat = Materials.AirbusEES.Duct,
    t=t_duct,
    A=A_duct,
    initOpt=environment.initOpt,
    Tstart=T_start)
    annotation (Placement(transformation(extent={{48,-36},{68,-16}})));
  Components.HeatTransfer.InternalConvection ductExtConduction(A=A_duct,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_duct))
    annotation (Placement(transformation(extent={{48,-48},{68,-28}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a ductThermalPort
    annotation (Placement(transformation(extent={{82,-52},{102,-32}})));
  // Floor
  Components.HeatTransfer.WallConduction floorConduction(
    t=t_floor,
    A=A_floor,
    redeclare model Mat = Materials.AirbusEES.Floor,
    Tstart=T_start,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{48,22},{68,42}})));
  Components.HeatTransfer.InternalConvection floorIntConvection(A=A_floor,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
         ht_fixed=ht_int_fixed))
    annotation (Placement(transformation(extent={{48,34},{68,54}})));
  Components.HeatTransfer.InternalConvection floorExtConvection(A=A_floor,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
         ht_fixed=ht_floor_fixed))
            annotation (Placement(transformation(extent={{48,10},{68,30}})));
  BoundaryConditions.thermal floorExtTempInput(T=T_floor, use_T=true)
    annotation (Placement(transformation(extent={{54,10},{60,14}})));
  // Window
  BoundaryConditions.thermal windowHeatInput(use_Q=true, Q=-E_sun*Ap_win*R_win)
    annotation (Placement(transformation(extent={{-32,48},{-26,52}})));
  Components.HeatTransfer.InternalConvection windowIntConvection(A=A_win,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_fixed))
            annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-24,38})));
  BoundaryConditions.thermal windowReflectiveInput(
    use_in_Q=false,
    use_Q=true,
    Q=W_vitre_ck)
    annotation (Placement(transformation(extent={{-32,54},{-26,58}})));
  // Fuselage
  Components.HeatTransfer.InternalConvection fuselageIntConvection(A=A_fus,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_fixed))
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-20,4})));
  Components.MassTransfer.Plenum plenum(
    V=A_floor*A_fus/2/LX,
    noInitialPressure=false,
    initOpt=environment.initOpt,
    noInitialTemperature=false,
    Q_int=Q_avionics,
    m_H2O_fixed=N_occupants*m_H2O,
    Q_sens_fixed=N_occupants*Q_occupants,
    N_occupants=N_occupants,
    allowFlowReversal=false,
    X_start=X_start,
    fixed_Q=false)
    annotation (Placement(transformation(extent={{-10,-80},{10,-60}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-40,-70},{-60,-50}})));
  Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{50,-80},{70,-60}})));
  Modelica.Blocks.Interfaces.RealOutput plenumTemp annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-110,80})));
  Modelica.Blocks.Interfaces.RealOutput plenumPressure
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  // Ports
  CustomInterfaces.FluidPort_A inletPort
    annotation (Placement(transformation(extent={{-34,-96},{-14,-76}})));
  CustomInterfaces.FluidPort_B outletPort
    annotation (Placement(transformation(extent={{10,-96},{30,-76}})));
  Components.HeatTransfer.WallConduction fusConduction(
    redeclare model Mat = Materials.AirbusEES.Fuselage,
    t=t_fus,
    A=A_fus,
    Tstart=T_start,
    initOpt=environment.initOpt) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-36,4})));
  Components.HeatTransfer.WallConduction windowConduction(
    redeclare model Mat = Materials.AirbusEES.Window,
    t=t_win,
    A=A_win,
    Tstart=T_start,
    initOpt=environment.initOpt)
             annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-40,38})));
  Components.HeatTransfer.ExternalConvection windowExtConvection(redeclare
      model HTC =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue (
         ht_fixed=ht_ext_fixed), A=A_win) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-56,38})));
  Components.HeatTransfer.ExternalConvection fuselageExtConvection(A=A_fus,
      redeclare model HTC =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue (
          ht_fixed=ht_ext_fixed)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-54,4})));
equation
  // Window Balance (has to be reviewed, awaiting Airbus reply)
//   W_ext + W_int = 0;
//   W_ext = A_win*ht_ext_fixed*(environment.T_amb-T_w_ext);
//   W_int = A_win*ht_int_fixed*(plenum.T-T_w_int);
//   W_ext - W_int = A_win*windowConduction.Mat.lambda/t_win*(T_w_ext-T_w_int);

  connect(floorIntConvection.outlet, floorConduction.inlet)
    annotation (Line(points={{58,40.6},{58,35.4}}, color={191,0,0}));
  connect(floorConduction.outlet, floorExtConvection.inlet)
    annotation (Line(points={{58,28.6},{58,23.4}},color={191,0,0}));
  connect(floorExtTempInput.thermal, floorExtConvection.outlet)
    annotation (Line(points={{58,12},{58,16.6}}, color={191,0,0}));
  connect(windowIntConvection.inlet, windowHeatInput.thermal) annotation (Line(
        points={{-20.6,38},{0,38},{0,50},{-28,50}}, color={191,0,0}));
  connect(fuselageIntConvection.inlet, windowHeatInput.thermal) annotation (Line(
        points={{-16.6,4},{0,4},{0,50},{-28,50}}, color={191,0,0}));
  connect(floorIntConvection.inlet, windowHeatInput.thermal) annotation (Line(
        points={{58,47.4},{58,56},{0,56},{0,50},{-28,50}}, color={191,0,0}));
  connect(outletPort, outletPort)
    annotation (Line(points={{20,-86},{20,-86}}, color={0,0,0}));
  connect(outletPort, plenum.outlet)
    annotation (Line(points={{20,-86},{20,-70},{10,-70}}, color={0,0,0}));
  connect(inletPort, inletPort)
    annotation (Line(points={{-24,-86},{-24,-86}}, color={0,0,0}));
  connect(plenum.inlet, inletPort) annotation (Line(points={{-10,-70},{-24,
          -70},{-24,-86}}, color={0,0,0}));
  connect(plenum.thermalPort, windowHeatInput.thermal)
    annotation (Line(points={{0,-61},{0,50},{-28,50}}, color={191,0,0}));
  connect(windowReflectiveInput.thermal, windowHeatInput.thermal) annotation (
      Line(points={{-28,56},{0,56},{0,50},{-28,50}}, color={191,0,0}));
  connect(plenum.thermalPort, temperatureSensor.port) annotation (Line(
        points={{0,-61},{-14,-61},{-14,-60},{-40,-60}}, color={191,0,0}));
  connect(pressureSensor.port, plenum.outlet)
    annotation (Line(points={{50,-70},{10,-70}}, color={0,0,0}));
  connect(pressureSensor.y, plenumPressure)
    annotation (Line(points={{71,-70},{80,-70},{80,80},{110,80}},
                                                 color={0,0,127}));
  connect(plenumTemp, plenumTemp)
    annotation (Line(points={{-110,80},{-110,80}}, color={0,0,127}));
  connect(temperatureSensor.T, plenumTemp)
    annotation (Line(points={{-61,-60},{-80,-60},{-80,80},{-110,80}},
                                                   color={0,0,127}));
  connect(ductConduction.inlet,ductIntConvection. outlet)
    annotation (Line(points={{58,-22.6},{58,-17.4}}, color={191,0,0}));
  connect(ductExtConduction.inlet,ductConduction. outlet)
    annotation (Line(points={{58,-34.6},{58,-29.4}}, color={191,0,0}));
  connect(ductIntConvection.inlet, windowHeatInput.thermal) annotation (
      Line(points={{58,-10.6},{58,4},{0,4},{0,50},{-28,50}}, color={191,0,0}));
  connect(ductThermalPort, ductExtConduction.outlet) annotation (Line(
        points={{92,-42},{58,-42},{58,-41.4}}, color={191,0,0}));
  connect(windowConduction.inlet, windowIntConvection.outlet)
    annotation (Line(points={{-36.6,38},{-27.4,38}}, color={191,0,0}));
  connect(fusConduction.inlet, fuselageIntConvection.outlet)
    annotation (Line(points={{-32.6,4},{-23.4,4}}, color={191,0,0}));
  connect(windowExtConvection.inlet, windowConduction.outlet)
    annotation (Line(points={{-52.6,38},{-43.4,38}}, color={191,0,0}));
  connect(fuselageExtConvection.inlet, fusConduction.outlet)
    annotation (Line(points={{-50.6,4},{-39.4,4}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{38,50},{78,10}},   lineColor={28,108,200}),
        Text(
          extent={{38,54},{60,50}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Floor"),
        Rectangle(extent={{-60,20},{-12,-10}}, lineColor={28,108,200}),
        Text(
          extent={{-60,26},{-44,20}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Fuselage"),
        Rectangle(extent={{-62,60},{-14,26}}, lineColor={28,108,200}),
        Text(
          extent={{-60,64},{-46,62}},
          lineColor={28,108,200},
          textString="Window",
          fontSize=10),
        Rectangle(extent={{36,-8},{76,-48}},  lineColor={28,108,200}),
        Text(
          extent={{32,-4},{54,-8}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Duct")}));
end AirbusCockpitDuct;
