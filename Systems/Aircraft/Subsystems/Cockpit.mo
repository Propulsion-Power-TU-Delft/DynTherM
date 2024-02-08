within DynTherM.Systems.Aircraft.Subsystems;
model Cockpit "Upper section of the fuselage: cockpit section"

  outer DynTherM.Components.Environment environment "Environmental properties";

  replaceable model HTC_int =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    annotation (choicesAllMatching=true);

  replaceable model HTC_ext =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    annotation (choicesAllMatching=true);

  parameter Real N_occupants[3] "Number of: passengers, cabin crew, pilots inside the cockpit";
  parameter Modelica.Units.SI.HeatFlowRate Q_int "Internal heat load";
  parameter Modelica.Units.SI.Length L_cockpit
    "Length of the cockpit cylindrical section";
  parameter Modelica.Units.SI.Length R_ext "External radius of the fuselage";
  parameter Modelica.Units.SI.Volume V_cockpit "Cockpit internal volume";
  parameter Modelica.Units.SI.SpecificHeatCapacity c_cockpit
    "Specific heat capacity of cockpit interior";
  parameter Modelica.Units.SI.Mass m_cockpit "Mass of cockpit interior";
  parameter Modelica.Units.SI.Area A_cockpit
    "Heat transfer surface of cockpit interior";
  parameter Modelica.Units.SI.Length L_windshield_front
    "Windshield length - frontal section";
  parameter Modelica.Units.SI.Length H_windshield_front
    "Windshield height - frontal section";
  parameter Modelica.Units.SI.Length L_windshield_lat
    "Windshield length - lateral section";
  parameter Modelica.Units.SI.Length H_windshield_lat
    "Windshield height - lateral section";
  parameter Modelica.Units.SI.Length t_cockpit
    "Overall fuselage thickness (cockpit section)";
  parameter Modelica.Units.SI.Irradiance E_tb_1
    "Beam component of the clear-sky solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_1
    "Diffuse component of the clear-sky solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_1
    "Ground reflected component of the clear-sky solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_1
    "Incidence angle - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tb_2
    "Beam component of the clear-sky solar irradiance - section 2" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_2
    "Diffuse component of the clear-sky solar irradiance - section 2" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_2
    "Ground reflected component of the clear-sky solar irradiance - section 2" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_2
    "Incidence angle - section 2" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tb_3
    "Beam component of the clear-sky solar irradiance - section 3" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_3
    "Diffuse component of the clear-sky solar irradiance - section 3" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_3
    "Ground reflected component of the clear-sky solar irradiance - section 3" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_3
    "Incidence angle - section 3" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tb_4
    "Beam component of the clear-sky solar irradiance - section 4" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_4
    "Diffuse component of the clear-sky solar irradiance - section 4" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_4
    "Ground reflected component of the clear-sky solar irradiance - section 4" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_4
    "Incidence angle - section 4" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tb_5
    "Beam component of the clear-sky solar irradiance - section 5" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_5
    "Diffuse component of the clear-sky solar irradiance - section 5" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_5
    "Ground reflected component of the clear-sky solar irradiance - section 5" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_5
    "Incidence angle - section 5" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tb_front
    "Beam component of the clear-sky solar irradiance - frontal section" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_front
    "Diffuse component of the clear-sky solar irradiance - frontal section" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_front
    "Ground reflected component of the clear-sky solar irradiance - frontal section" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_front
    "Incidence angle - frontal section" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Temperature Tstart_fuselage
    "Fuselage temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature Tstart_flightDeck
    "Cabin temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure Pstart_flightDeck
    "Cabin pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature=false "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  final parameter Modelica.Units.SI.Length L_oct=section_1.R_ext*sqrt(2 - sqrt(
      2)) "Approximated length of one fuselage section (octagon)";
  final parameter Modelica.Units.SI.Length W_fl=L_oct*(1 + sqrt(2))
    "Floor Width";

  Components.MassTransfer.Plenum flightDeck(
    V=V_cockpit,
    N_occupants=N_occupants,
    Q_int=Q_int,
    P_start=Pstart_flightDeck,
    T_start=Tstart_flightDeck,
    noInitialPressure=noInitialPressure,
    noInitialTemperature=noInitialTemperature,
    fixed_Q=false)
    annotation (Placement(transformation(extent={{20,-80},{-20,-40}})));
  CustomInterfaces.FluidPort_A cockpitInflow annotation (Placement(
        transformation(extent={{94,-86},{106,-74}}), iconTransformation(extent=
            {{50,50},{70,70}})));
  CustomInterfaces.FluidPort_B cockpitToCargo annotation (Placement(
        transformation(extent={{-106,-86},{-94,-74}}), iconTransformation(
          extent={{50,-70},{70,-50}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a cockpitToCabin
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
        iconTransformation(extent={{90,-10},{110,10}})));
  Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{66,-72},{90,-48}})));
  Modelica.Blocks.Interfaces.RealOutput cockpitPressure annotation (Placement(
        transformation(extent={{94,-72},{118,-48}}), iconTransformation(extent={{-92,-42},
            {-116,-18}})));

  DynTherM.Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_3(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    coeff=1/8,
    L_fuselage=L_cockpit,
    R_ext=R_ext,
    t_fuselage=t_cockpit,
    csi=0,
    E_tb=E_tb_3,
    E_td=E_td_3,
    E_tr=E_tr_3,
    theta=theta_3,
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-50,62},{-10,102}})));

  DynTherM.Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_2(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    coeff=1/8,
    L_fuselage=L_cockpit,
    R_ext=R_ext,
    t_fuselage=t_cockpit,
    csi=5.4977871437821,
    E_tb=E_tb_2,
    E_td=E_td_2,
    E_tr=E_tr_2,
    theta=theta_2,
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{94,62},{54,102}})));
  DynTherM.Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_4(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    coeff=1/8,
    L_fuselage=L_cockpit,
    R_ext=R_ext,
    t_fuselage=t_cockpit,
    csi=0.78539816339745,
    E_tb=E_tb_4,
    E_td=E_td_4,
    E_tr=E_tr_4,
    theta=theta_4,
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-94,62},{-54,102}})));

  Modelica.Blocks.Interfaces.RealOutput cockpitTemperature annotation (
      Placement(transformation(extent={{94,-42},{118,-18}}), iconTransformation(
          extent={{-92,-18},{-116,6}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{66,-40},{86,-20}})));
  Components.HeatTransfer.HeatCapacity cockpitInterior(
    initOpt=environment.initOpt,                       T_start=
        Tstart_flightDeck,
    C=m_cockpit*c_cockpit)
    annotation (Placement(transformation(extent={{-26,20},{-2,44}})));
  Components.HeatTransfer.InternalConvection internalConvection(
    redeclare model HTC=HTC_int, A=A_cockpit)
    annotation (Placement(transformation(extent={{-24,-2},{-4,-22}})));
  Components.HeatTransfer.SolarRadiation solarRadiation_front(
    E_tb_fixed=E_tb_front,
    E_td_fixed=E_td_front,
    E_tr_fixed=E_tr_front,
    theta_fixed=theta_front,
    csi=0.78539816339745,
    psi_plus=1.5707963267949)
    annotation (Placement(transformation(extent={{-2,98},{30,66}})));
  Components.HeatTransfer.ExternalConvection extConvectionWindow_front(A=
        L_windshield_front*H_windshield_front,
      redeclare model HTC = HTC_ext)
    annotation (Placement(transformation(extent={{26,84},{46,64}})));
  Components.HeatTransfer.InternalConvection internalConvection_front(A=
        L_windshield_front*H_windshield_front,
      redeclare model HTC = HTC_int)
    annotation (Placement(transformation(extent={{10,-2},{30,-22}})));
  FuselageHeatTransferWindow
    section_5(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    coeff=1/8,
    L_fuselage=L_cockpit,
    R_ext=R_ext,
    t_fuselage=t_cockpit,
    csi=1.5707963267949,
    E_tb=E_tb_5,
    E_td=E_td_5,
    E_tr=E_tr_5,
    theta=theta_5,
    L_window=L_windshield_lat,
    H_window=H_windshield_lat,
    Nw_side=1,
    Tstart_fuselage=Tstart_fuselage) annotation (Placement(transformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={-82,14})));
  FuselageHeatTransferWindow
    section_1(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    coeff=1/8,
    L_fuselage=L_cockpit,
    R_ext=R_ext,
    t_fuselage=t_cockpit,
    csi=4.7123889803847,
    E_tb=E_tb_1,
    E_td=E_td_1,
    E_tr=E_tr_1,
    theta=theta_1,
    L_window=L_windshield_lat,
    H_window=H_windshield_lat,
    Nw_side=1,
    Tstart_fuselage=Tstart_fuselage) annotation (Placement(transformation(
        extent={{20,20},{-20,-20}},
        rotation=90,
        origin={82,14})));
  FlightDeckWindshield windshieldFront(
    A_windshield=L_windshield_front*H_windshield_front,
    Tstart=Tstart_fuselage)
    annotation (Placement(transformation(extent={{2,34},{32,66}})));
equation
  section_5.P_air_gap = cockpitInflow.P;
  section_5.X_air_gap = cockpitInflow.Xi_outflow;
  section_1.P_air_gap = cockpitInflow.P;
  section_1.X_air_gap = cockpitInflow.Xi_outflow;

  connect(cockpitToCargo, flightDeck.outlet)
    annotation (Line(points={{-100,-80},{-20,-80},{-20,-60}}, color={0,0,0}));
  connect(flightDeck.thermalPort, cockpitToCabin)
    annotation (Line(points={{0,-42},{0,-100}}, color={191,0,0}));
  connect(flightDeck.inlet, pressureSensor.port)
    annotation (Line(points={{20,-60},{66,-60}}, color={0,0,0}));
  connect(flightDeck.inlet, cockpitInflow)
    annotation (Line(points={{20,-60},{20,-80},{100,-80}}, color={0,0,0}));
  connect(section_2.heatToInner, flightDeck.thermalPort) annotation (Line(
        points={{74,70},{74,60},{60,60},{60,-42},{0,-42}}, color={191,0,0}));
  connect(section_4.heatToInner, flightDeck.thermalPort) annotation (Line(
        points={{-74,70},{-74,60},{-60,60},{-60,-42},{0,-42}}, color={191,0,0}));
  connect(section_3.heatToInner, flightDeck.thermalPort) annotation (Line(
        points={{-30,70},{-30,-42},{0,-42}},     color={191,0,0}));
  connect(pressureSensor.y, cockpitPressure)
    annotation (Line(points={{91.2,-60},{106,-60}}, color={0,0,127}));
  connect(flightDeck.thermalPort, temperatureSensor.port)
    annotation (Line(points={{0,-42},{0,-30},{66,-30}}, color={191,0,0}));
  connect(temperatureSensor.T, cockpitTemperature)
    annotation (Line(points={{87,-30},{106,-30}}, color={0,0,127}));
  connect(internalConvection.inlet, flightDeck.thermalPort)
    annotation (Line(points={{-14,-15.4},{-14,-30},{0,-30},{0,-42}},
                                                           color={191,0,0}));
  connect(internalConvection_front.inlet, flightDeck.thermalPort)
    annotation (Line(points={{20,-15.4},{20,-30},{0,-30},{0,-42}},
                                                         color={191,0,0}));
  connect(section_5.heatAbsorbed, flightDeck.thermalPort) annotation (Line(
        points={{-70,8},{-60,8},{-60,-42},{0,-42}}, color={191,0,0}));
  connect(section_1.heatAbsorbed, flightDeck.thermalPort) annotation (Line(
        points={{70,8},{60,8},{60,-42},{0,-42}}, color={191,0,0}));
  connect(windshieldFront.irradianceExt, solarRadiation_front.inlet)
    annotation (Line(points={{14,54.9231},{14,72.08}}, color={191,0,0}));
  connect(windshieldFront.heatExt, extConvectionWindow_front.inlet) annotation (
     Line(points={{20,54.9231},{20,64},{36,64},{36,70.6}}, color={191,0,0}));
  connect(windshieldFront.heatAbsorbed, internalConvection_front.outlet)
    annotation (Line(points={{20,45.0769},{20,-8.6}}, color={191,0,0}));
  connect(cockpitInterior.port, internalConvection.outlet) annotation (Line(
        points={{-14,20},{-14,-8.6}},             color={191,0,0}));
  connect(section_5.heatTransmitted, cockpitInterior.port)
    annotation (Line(points={{-70,20},{-14,20}}, color={191,0,0}));
  connect(section_1.heatTransmitted, cockpitInterior.port)
    annotation (Line(points={{70,20},{-14,20}}, color={191,0,0}));
  connect(windshieldFront.heatTransmitted, cockpitInterior.port)
    annotation (Line(points={{14,45.0769},{14,20},{-14,20}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-100,-60},{100,26}},
          lineColor={0,0,0},
          lineThickness=0.5,
          closure=EllipseClosure.None,
          startAngle=122,
          endAngle=270),
        Ellipse(
          extent={{-72,-104},{200,60}},
          lineColor={0,0,0},
          lineThickness=0.5,
          closure=EllipseClosure.None,
          startAngle=90,
          endAngle=109),
        Line(
          points={{-80,-42},{-80,8}},
          color={0,0,0},
          thickness=0.5),
        Ellipse(
          extent={{-100,-60},{100,26}},
          lineColor={85,170,255},
          lineThickness=0.5,
          closure=EllipseClosure.None,
          startAngle=90,
          endAngle=122),
        Ellipse(
          extent={{-72,-104},{200,60}},
          lineColor={85,170,255},
          lineThickness=0.5,
          closure=EllipseClosure.None,
          startAngle=100,
          endAngle=150),
        Line(
          points={{100,26},{40,26}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{20,26},{20,56}},
          color={85,170,255},
          thickness=0.5),
        Line(
          points={{20,26},{0,26}},
          color={85,170,255},
          thickness=0.5),
        Line(
          points={{40,26},{40,58}},
          color={85,170,255},
          thickness=0.5),
        Line(
          points={{40,26},{20,26}},
          color={85,170,255},
          thickness=0.5),
        Line(
          points={{0,-60},{100,-60},{100,60},{60,60}},
          color={0,0,0},
          thickness=0.5)}),                                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Cockpit;
