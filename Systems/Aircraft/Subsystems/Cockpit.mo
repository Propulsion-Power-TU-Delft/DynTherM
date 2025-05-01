within DynTherM.Systems.Aircraft.Subsystems;
model Cockpit "Upper section of the fuselage: cockpit section"

  outer Components.Environment environment "Environmental properties";

  replaceable model Paint =
    Materials.Paints.WhiteCoatings.CatalacWhitePaint
    constrainedby Materials.Paints.BasePaint "Surface paint material" annotation (choicesAllMatching=true);

  replaceable model HTC_int =
    Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    Components.HeatTransfer.HTCorrelations.BaseClassInternal
    "Internal convection correlation" annotation (choicesAllMatching=true);

  replaceable model HTC_ext =
    Components.HeatTransfer.HTCorrelations.BaseClassExternal
    constrainedby
    Components.HeatTransfer.HTCorrelations.BaseClassExternal
    "External convection correlation" annotation (choicesAllMatching=true);

  parameter Real N_occupants[3] "Number of: passengers, cabin crew, pilots inside the cockpit";
  parameter HeatFlowRate Q_int "Internal heat load";
  parameter Length L_cockpit "Length of the cockpit cylindrical section";
  parameter Length R_ext "External radius of the fuselage";
  parameter Volume V_cockpit "Cockpit internal volume";
  parameter SpecificHeatCapacity c_cockpit "Specific heat capacity of cockpit interior";
  parameter Mass m_cockpit "Mass of cockpit interior";
  parameter Area A_cockpit "Heat transfer surface of cockpit interior";
  parameter Length L_windshield_front "Windshield length - frontal section";
  parameter Length H_windshield_front "Windshield height - frontal section";
  parameter Length L_windshield_lat "Windshield length - lateral section";
  parameter Length H_windshield_lat "Windshield height - lateral section";
  parameter Length t_cockpit "Overall fuselage thickness (cockpit section)";

  // Radiation
  parameter Real rho_g=0.2 "Ground reflectance" annotation (Dialog(tab="Radiation"));
  parameter Angle csi[6]={
    Modelica.Units.Conversions.from_deg(270),
    Modelica.Units.Conversions.from_deg(315),
    Modelica.Units.Conversions.from_deg(0),
    Modelica.Units.Conversions.from_deg(45),
    Modelica.Units.Conversions.from_deg(90),
    Modelica.Units.Conversions.from_deg(45)}
    "Tilt angle of the surface wrt horizontal - sections 1-5 + front" annotation (Dialog(tab="Radiation"));
  parameter Angle psi_plus=0 "Modifier of azimuth angle" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_tb[5]
    "Beam component of the clear-sky solar irradiance - sections 1-5" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_td[5]
    "Diffuse component of the clear-sky solar irradiance - sections 1-5" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_tr[5]
    "Ground reflected component of the clear-sky solar irradiance - sections 1-5" annotation (Dialog(tab="Radiation"));
  parameter Angle theta[5] "Incidence angle - sections 1-5" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_tb_front=0
    "Beam component of the clear-sky solar irradiance - frontal section" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_td_front=0
    "Diffuse component of the clear-sky solar irradiance - frontal section" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_tr_front=0
    "Ground reflected component of the clear-sky solar irradiance - frontal section" annotation (Dialog(tab="Radiation"));
  parameter Angle theta_front=0 "Incidence angle - frontal section" annotation (Dialog(tab="Radiation"));

  // Initialization
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_fuselage "Fuselage temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_flightDeck "Cabin temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure Pstart_flightDeck "Cabin pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature=false "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  Length L_oct "Approximated length of one fuselage section (octagon)";
  Length W_fl "Floor Width";

  Components.MassTransfer.Plenum flightDeck(
    initOpt=initOpt,
    V=V_cockpit,
    N_occupants=N_occupants,
    Q_int=Q_int,
    P_start=Pstart_flightDeck,
    T_start=Tstart_flightDeck,
    noInitialPressure=noInitialPressure,
    noInitialTemperature=noInitialTemperature,
    fixed_Q=false)
    annotation (Placement(transformation(extent={{20,-80},{-20,-40}})));
  CustomInterfaces.ZeroDimensional.FluidPort_A cockpitInflow annotation (
      Placement(transformation(extent={{94,-86},{106,-74}}), iconTransformation(
          extent={{50,50},{70,70}})));
  CustomInterfaces.ZeroDimensional.FluidPort_B cockpitToCargo annotation (
      Placement(transformation(extent={{-106,-86},{-94,-74}}),
        iconTransformation(extent={{50,-70},{70,-50}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a cockpitToCabin
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
        iconTransformation(extent={{90,-10},{110,10}})));
  Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{66,-72},{90,-48}})));
  Modelica.Blocks.Interfaces.RealOutput cockpitPressure annotation (Placement(
        transformation(extent={{94,-72},{118,-48}}), iconTransformation(extent={{-92,-42},
            {-116,-18}})));

  Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_3(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cockpit,
    R_ext=R_ext,
    t_fuselage=t_cockpit,
    rho_g=rho_g,
    csi=csi[3],
    psi_plus=psi_plus,
    E_tb=E_tb[3],
    E_td=E_td[3],
    E_tr=E_tr[3],
    theta=theta[3],
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-50,62},{-10,102}})));

  Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_2(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cockpit,
    R_ext=R_ext,
    t_fuselage=t_cockpit,
    rho_g=rho_g,
    csi=csi[2],
    psi_plus=psi_plus,
    E_tb=E_tb[2],
    E_td=E_td[2],
    E_tr=E_tr[2],
    theta=theta[2],
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{94,62},{54,102}})));
  Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_4(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cockpit,
    R_ext=R_ext,
    t_fuselage=t_cockpit,
    rho_g=rho_g,
    csi=csi[4],
    psi_plus=psi_plus,
    E_tb=E_tb[4],
    E_td=E_td[4],
    E_tr=E_tr[4],
    theta=theta[4],
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-94,62},{-54,102}})));

  Modelica.Blocks.Interfaces.RealOutput cockpitTemperature annotation (
      Placement(transformation(extent={{94,-42},{118,-18}}), iconTransformation(
          extent={{-92,-18},{-116,6}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{66,-40},{86,-20}})));
  Components.HeatTransfer.HeatCapacity cockpitInterior(
    initOpt=initOpt,
    T_start=Tstart_flightDeck,
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
    rho_g=rho_g,
    csi=csi[6],
    psi_plus=1.5707963267949)
    annotation (Placement(transformation(extent={{-2,98},{30,66}})));
  Components.HeatTransfer.ExternalConvection extConvectionWindow_front(
    redeclare model HTC = HTC_ext,
    A=L_windshield_front*H_windshield_front)
    annotation (Placement(transformation(extent={{26,84},{46,64}})));
  Components.HeatTransfer.InternalConvection internalConvection_front(
    redeclare model HTC = HTC_int,
    A=L_windshield_front*H_windshield_front)
    annotation (Placement(transformation(extent={{10,-2},{30,-22}})));
  FuselageHeatTransferWindow
    section_5(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cockpit,
    R_ext=R_ext,
    t_fuselage=t_cockpit,
    rho_g=rho_g,
    csi=csi[5],
    psi_plus=psi_plus,
    E_tb=E_tb[5],
    E_td=E_td[5],
    E_tr=E_tr[5],
    theta=theta[5],
    L_window=L_windshield_lat,
    H_window=H_windshield_lat,
    Nw_side=1,
    Tstart_fuselage=Tstart_fuselage) annotation (Placement(transformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={-82,14})));
  FuselageHeatTransferWindow
    section_1(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cockpit,
    R_ext=R_ext,
    t_fuselage=t_cockpit,
    rho_g=rho_g,
    csi=csi[1],
    psi_plus=psi_plus,
    E_tb=E_tb[1],
    E_td=E_td[1],
    E_tr=E_tr[1],
    theta=theta[1],
    L_window=L_windshield_lat,
    H_window=H_windshield_lat,
    Nw_side=1,
    Tstart_fuselage=Tstart_fuselage) annotation (Placement(transformation(
        extent={{20,20},{-20,-20}},
        rotation=90,
        origin={82,14})));
  FlightDeckWindshield windshieldFront(
    initOpt=initOpt,
    A_windshield=L_windshield_front*H_windshield_front,
    Tstart=Tstart_fuselage)
    annotation (Placement(transformation(extent={{2,34},{32,66}})));

equation
  section_5.P_air_gap = cockpitInflow.P;
  section_5.X_air_gap = cockpitInflow.Xi_outflow;
  section_1.P_air_gap = cockpitInflow.P;
  section_1.X_air_gap = cockpitInflow.Xi_outflow;

  L_oct = section_1.R_ext*sqrt(2 - sqrt(2));
  W_fl = L_oct*(1 + sqrt(2));

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
