within DynTherM.Systems.Aircraft.Subsystems;
model PassengerCabin "Upper section of the fuselage: cabin section"

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

  parameter Real N_occupants[3] "Number of: passengers, cabin crew, pilots inside the cabin";
  parameter Modelica.Units.SI.HeatFlowRate Q_int "Internal heat load";
  parameter Modelica.Units.SI.Length L_fuselage
    "Length of the fuselage cylindrical section";
  parameter Modelica.Units.SI.Length R_ext "External radius of the fuselage";
  parameter Modelica.Units.SI.Volume V_cabin "Passenger cabin internal volume";
  parameter Modelica.Units.SI.SpecificHeatCapacity c_cabin
    "Specific heat capacity of cabin interior";
  parameter Modelica.Units.SI.Mass m_cabin "Mass of cabin interior";
  parameter Modelica.Units.SI.Area A_cabin
    "Heat transfer surface of cabin interior";
  parameter Modelica.Units.SI.Area A_floor "Surface area of cabin floor";
  parameter Modelica.Units.SI.Length L_window "Window length";
  parameter Modelica.Units.SI.Length H_window "Window height";
  parameter Integer Nw_side "Number of windows per fuselage side";
  parameter Modelica.Units.SI.Length t_cabin
    "Overall fuselage thickness (cabin section)";
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
  parameter Modelica.Units.SI.Temperature Tstart_fuselage
    "Fuselage temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature Tstart_cabin
    "Cabin temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure Pstart_cabin
    "Cabin pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature=false "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  final parameter Modelica.Units.SI.Length L_oct=section_1.R_ext*sqrt(2 - sqrt(
      2)) "Approximated length of one fuselage section (octagon)";
  final parameter Modelica.Units.SI.Length W_fl=L_oct*(1 + sqrt(2))
    "Floor Width";

  Components.MassTransfer.Plenum cabin(
    V=V_cabin,
    N_occupants=N_occupants,
    Q_int=Q_int,
    P_start=Pstart_cabin,
    T_start=Tstart_cabin,
    noInitialPressure=noInitialPressure,
    noInitialTemperature=noInitialTemperature,
    fixed_Q=false)
    annotation (Placement(transformation(extent={{20,-80},{-20,-40}})));
  CustomInterfaces.FluidPort_A cabinInflow annotation (Placement(transformation(
          extent={{94,-86},{106,-74}}), iconTransformation(extent={{-10,90},{10,
            110}})));
  CustomInterfaces.FluidPort_B cabinToCargo annotation (Placement(
        transformation(extent={{-106,-86},{-94,-74}}), iconTransformation(
          extent={{-10,-110},{10,-90}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a cabinToFloor annotation (
      Placement(transformation(extent={{-10,-110},{10,-90}}),
        iconTransformation(extent={{-110,-20},{-90,0}})));
  Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{66,-72},{90,-48}})));
  Modelica.Blocks.Interfaces.RealOutput cabinPressure annotation (Placement(
        transformation(extent={{94,-72},{118,-48}}), iconTransformation(
          extent={{88,-42},{112,-18}})));

  DynTherM.Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_3(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    coeff=1/8,
    L_fuselage=L_fuselage,
    R_ext=R_ext,
    t_fuselage=t_cabin,
    csi=0,
    E_tb=E_tb_3,
    E_td=E_td_3,
    E_tr=E_tr_3,
    theta=theta_3,
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-20,62},{20,102}})));

  DynTherM.Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_2(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    coeff=1/8,
    L_fuselage=L_fuselage,
    R_ext=R_ext,
    t_fuselage=t_cabin,
    csi=5.4977871437821,
    E_tb=E_tb_2,
    E_td=E_td_2,
    E_tr=E_tr_2,
    theta=theta_2,
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{80,62},{40,102}})));
  DynTherM.Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_4(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    coeff=1/8,
    L_fuselage=L_fuselage,
    R_ext=R_ext,
    t_fuselage=t_cabin,
    csi=0.78539816339745,
    E_tb=E_tb_4,
    E_td=E_td_4,
    E_tr=E_tr_4,
    theta=theta_4,
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-80,62},{-40,102}})));
  DynTherM.Systems.Aircraft.Subsystems.FuselageHeatTransferWindow section_5(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    coeff=1/8,
    L_fuselage=L_fuselage,
    R_ext=R_ext,
    t_fuselage=t_cabin,
    csi=1.5707963267949,
    E_tb=E_tb_5,
    E_td=E_td_5,
    E_tr=E_tr_5,
    theta=theta_5,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart_fuselage=Tstart_fuselage) annotation (Placement(transformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={-82,14})));
  DynTherM.Systems.Aircraft.Subsystems.FuselageHeatTransferWindow section_1(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    coeff=1/8,
    L_fuselage=L_fuselage,
    R_ext=R_ext,
    t_fuselage=t_cabin,
    csi=4.7123889803847,
    E_tb=E_tb_1,
    E_td=E_td_1,
    E_tr=E_tr_1,
    theta=theta_1,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart_fuselage=Tstart_fuselage) annotation (Placement(transformation(
        extent={{20,20},{-20,-20}},
        rotation=90,
        origin={82,14})));

  Modelica.Blocks.Interfaces.RealOutput cabinTemperature annotation (
      Placement(transformation(extent={{94,-42},{118,-18}}),
        iconTransformation(extent={{88,-2},{112,22}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{66,-40},{86,-20}})));
  Components.HeatTransfer.HeatCapacity cabinInterior(
    initOpt=environment.initOpt,
    T_start=Tstart_cabin,
    C=m_cabin*c_cabin)
    annotation (Placement(transformation(extent={{-34,20},{-6,48}})));
  Components.HeatTransfer.InternalConvection internalConvection(
    redeclare model HTC=HTC_int,
    A=A_cabin)
    annotation (Placement(transformation(extent={{-32,0},{-8,-22}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier1(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{-28,-32},{-12,-18}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier2(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{-28,10},{-12,-4}})));
equation
  section_5.P_air_gap = cabinInflow.P;
  section_5.X_air_gap = cabinInflow.Xi_outflow;
  section_1.P_air_gap = cabinInflow.P;
  section_1.X_air_gap = cabinInflow.Xi_outflow;

  connect(cabinToCargo, cabin.outlet)
    annotation (Line(points={{-100,-80},{-20,-80},{-20,-60}}, color={0,0,0}));
  connect(cabin.thermalPort, cabinToFloor)
    annotation (Line(points={{0,-42},{0,-100}}, color={191,0,0}));
  connect(cabin.inlet, pressureSensor.port)
    annotation (Line(points={{20,-60},{66,-60}}, color={0,0,0}));
  connect(cabin.inlet, cabinInflow)
    annotation (Line(points={{20,-60},{20,-80},{100,-80}}, color={0,0,0}));
  connect(section_5.heatAbsorbed, cabin.thermalPort) annotation (Line(points={{
          -70,8},{-60,8},{-60,-42},{0,-42}}, color={191,0,0}));
  connect(section_1.heatAbsorbed, cabin.thermalPort) annotation (Line(points={{
          70,8},{60,8},{60,-42},{0,-42}}, color={191,0,0}));
  connect(section_2.heatToInner, cabin.thermalPort)
    annotation (Line(points={{60,70},{60,-42},{0,-42}}, color={191,0,0}));
  connect(section_4.heatToInner, cabin.thermalPort)
    annotation (Line(points={{-60,70},{-60,-42},{0,-42}}, color={191,0,0}));
  connect(section_3.heatToInner, cabin.thermalPort)
    annotation (Line(points={{0,70},{0,-42},{0,-42}}, color={191,0,0}));
  connect(pressureSensor.y, cabinPressure)
    annotation (Line(points={{91.2,-60},{106,-60}}, color={0,0,127}));
  connect(cabin.thermalPort, temperatureSensor.port)
    annotation (Line(points={{0,-42},{0,-30},{66,-30}}, color={191,0,0}));
  connect(temperatureSensor.T, cabinTemperature)
    annotation (Line(points={{87,-30},{106,-30}}, color={0,0,127}));
  connect(internalConvection.inlet, heatFlowMultiplier1.distributed)
    annotation (Line(points={{-20,-13.2},{-20,-20.8}},
                                                     color={191,0,0}));
  connect(heatFlowMultiplier1.single, cabin.thermalPort)
    annotation (Line(points={{-20,-29.2},{-20,-42},{0,-42}}, color={191,0,0}));
  connect(section_5.heatTransmitted, cabinInterior.port)
    annotation (Line(points={{-70,20},{-20,20}}, color={191,0,0}));
  connect(cabinInterior.port, section_1.heatTransmitted)
    annotation (Line(points={{-20,20},{70,20}}, color={191,0,0}));
  connect(cabinInterior.port, heatFlowMultiplier2.single)
    annotation (Line(points={{-20,20},{-20,7.2}}, color={191,0,0}));
  connect(heatFlowMultiplier2.distributed, internalConvection.outlet)
    annotation (Line(points={{-20,-1.2},{-20,-8.8}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          lineThickness=0.5),
        Rectangle(extent={{-70,50},{-50,10}}, lineColor={0,0,0}),
        Rectangle(extent={{-40,50},{-20,10}}, lineColor={0,0,0}),
        Rectangle(extent={{20,50},{40,10}}, lineColor={0,0,0}),
        Rectangle(extent={{50,50},{70,10}}, lineColor={0,0,0}),
        Line(points={{-70,20},{-50,20}}, color={0,0,0}),
        Line(points={{-40,20},{-20,20}}, color={0,0,0}),
        Line(points={{20,20},{40,20}}, color={0,0,0}),
        Line(points={{50,20},{70,20}}, color={0,0,0}),
        Line(
          points={{-32,92},{-16,88},{-6,72},{-6,26},{-10,6},{-26,-2},{-44,2},{-56,
              14},{-58,40}},
          color={28,108,200},
          smooth=Smooth.Bezier),
        Line(
          points={{-58,40},{-62,32}},
          color={28,108,200},
          smooth=Smooth.Bezier),
        Line(points={{-58,40},{-52,32}}, color={28,108,200}),
        Line(points={{60,40},{54,32}}, color={28,108,200}),
        Line(
          points={{60,40},{64,32}},
          color={28,108,200},
          smooth=Smooth.Bezier),
        Line(
          points={{32,92},{18,88},{8,72},{8,26},{12,6},{28,-2},{46,2},{58,14},{60,
              40}},
          color={28,108,200},
          smooth=Smooth.Bezier),
        Line(
          points={{-98,-10},{98,-10}},
          color={0,0,0},
          thickness=0.5)}),                                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PassengerCabin;
