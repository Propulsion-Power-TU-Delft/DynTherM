within DynTherM.Systems.Aircraft.Subsystems;
model PassengerCabin "Upper section of the fuselage: cabin section"

  outer Components.Environment environment "Environmental properties";

  replaceable model Paint =
    Materials.Paints.WhiteCoatings.CatalacWhitePaint
    constrainedby Materials.Paints.BasePaint "Surface paint material" annotation (choicesAllMatching=true);

  replaceable model HTC_int =
    Components.HeatTransfer.HTCorrelations.BaseClasses.BaseClassInternal
    constrainedby
    Components.HeatTransfer.HTCorrelations.BaseClasses.BaseClassInternal
    "Internal convection correlation" annotation (choicesAllMatching=true);

  replaceable model HTC_ext =
    Components.HeatTransfer.HTCorrelations.BaseClasses.BaseClassExternal
    constrainedby
    Components.HeatTransfer.HTCorrelations.BaseClasses.BaseClassExternal
    "External convection correlation" annotation (choicesAllMatching=true);

  parameter Real N_occupants[3] "Number of: passengers, cabin crew, pilots inside the cabin";
  input HeatFlowRate Q_int "Internal heat load" annotation (Dialog(enable=true));
  parameter Length L_cabin "Length of the fuselage cylindrical section";
  parameter Length R_ext "External radius of the fuselage";
  parameter Volume V_cabin "Passenger cabin internal volume";
  parameter SpecificHeatCapacity c_cabin "Specific heat capacity of cabin interior";
  parameter Mass m_cabin "Mass of cabin interior";
  parameter Area A_cabin "Heat transfer surface of cabin interior";
  input Area A_floor "Surface area of cabin floor" annotation (Dialog(enable=true));
  parameter Length L_window "Window length";
  parameter Length H_window "Window height";
  parameter Integer Nw_side "Number of windows per fuselage side";
  parameter Length t_cabin "Overall fuselage thickness (cabin section)";

  // Radiation
  parameter Real rho_g=0.2 "Ground reflectance" annotation (Dialog(tab="Radiation"));
  parameter Angle csi[5]={
    Modelica.Units.Conversions.from_deg(270),
    Modelica.Units.Conversions.from_deg(315),
    Modelica.Units.Conversions.from_deg(0),
    Modelica.Units.Conversions.from_deg(45),
    Modelica.Units.Conversions.from_deg(90)}
    "Tilt angle of the surface wrt horizontal - sections 1-5" annotation (Dialog(tab="Radiation"));
  parameter Angle psi_plus=0 "Modifier of azimuth angle" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_tb[5]
    "Beam component of the clear-sky solar irradiance - sections 1-5" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_td[5]
    "Diffuse component of the clear-sky solar irradiance - sections 1-5" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_tr[5]
    "Ground reflected component of the clear-sky solar irradiance - sections 1-5" annotation (Dialog(tab="Radiation"));
  parameter Angle theta[5] "Incidence angle - sections 1-5" annotation (Dialog(tab="Radiation"));

  // Initialization
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_fuselage "Fuselage temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_cabin "Cabin temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure Pstart_cabin "Cabin pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature=false "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  Length L_oct "Approximated length of one fuselage section (octagon)";
  Length W_fl "Floor Width";

  Components.MassTransfer.Plenum cabin(
    initOpt=initOpt,
    V=V_cabin,
    N_occupants=N_occupants,
    Q_int=Q_int,
    P_start=Pstart_cabin,
    T_start=Tstart_cabin,
    noInitialPressure=noInitialPressure,
    noInitialTemperature=noInitialTemperature,
    fixed_Q=false)
    annotation (Placement(transformation(extent={{20,-80},{-20,-40}})));
  CustomInterfaces.ZeroDimensional.FluidPort_A cabinInflow annotation (
      Placement(transformation(extent={{94,-86},{106,-74}}), iconTransformation(
          extent={{-10,90},{10,110}})));
  CustomInterfaces.ZeroDimensional.FluidPort_B cabinToCargo annotation (
      Placement(transformation(extent={{-106,-86},{-94,-74}}),
        iconTransformation(extent={{-10,-110},{10,-90}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a cabinToFloor annotation (
      Placement(transformation(extent={{-10,-110},{10,-90}}),
        iconTransformation(extent={{-110,-20},{-90,0}})));
  Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{66,-72},{90,-48}})));
  Modelica.Blocks.Interfaces.RealOutput cabinPressure annotation (Placement(
        transformation(extent={{94,-72},{118,-48}}), iconTransformation(
          extent={{88,-42},{112,-18}})));

  Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_3(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cabin,
    R_ext=R_ext,
    t_fuselage=t_cabin,
    rho_g=rho_g,
    csi=csi[3],
    psi_plus=psi_plus,
    E_tb=E_tb[3],
    E_td=E_td[3],
    E_tr=E_tr[3],
    theta=theta[3],
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-20,62},{20,102}})));

  Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_2(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cabin,
    R_ext=R_ext,
    t_fuselage=t_cabin,
    rho_g=rho_g,
    csi=csi[2],
    psi_plus=psi_plus,
    E_tb=E_tb[2],
    E_td=E_td[2],
    E_tr=E_tr[2],
    theta=theta[2],
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{80,62},{40,102}})));
  Systems.Aircraft.Subsystems.UpperFuselageHeatTransfer section_4(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cabin,
    R_ext=R_ext,
    t_fuselage=t_cabin,
    rho_g=rho_g,
    csi=csi[4],
    psi_plus=psi_plus,
    E_tb=E_tb[4],
    E_td=E_td[4],
    E_tr=E_tr[4],
    theta=theta[4],
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-80,62},{-40,102}})));
  Systems.Aircraft.Subsystems.FuselageHeatTransferWindow section_5(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cabin,
    R_ext=R_ext,
    t_fuselage=t_cabin,
    rho_g=rho_g,
    csi=csi[5],
    psi_plus=psi_plus,
    E_tb=E_tb[5],
    E_td=E_td[5],
    E_tr=E_tr[5],
    theta=theta[5],
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart_fuselage=Tstart_fuselage) annotation (Placement(transformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={-82,14})));
  Systems.Aircraft.Subsystems.FuselageHeatTransferWindow section_1(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cabin,
    R_ext=R_ext,
    t_fuselage=t_cabin,
    rho_g=rho_g,
    csi=csi[1],
    psi_plus=psi_plus,
    E_tb=E_tb[1],
    E_td=E_td[1],
    E_tr=E_tr[1],
    theta=theta[1],
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
    initOpt=initOpt,                                 T_start=Tstart_cabin,
    C=m_cabin*c_cabin)
    annotation (Placement(transformation(extent={{-32,20},{-8,44}})));
  Components.HeatTransfer.InternalConvection internalConvection(
    redeclare model HTC=HTC_int,
    A=A_cabin)
    annotation (Placement(transformation(extent={{-30,-6},{-10,-26}})));

equation
  section_5.P_air_gap = cabinInflow.P;
  section_5.X_air_gap = cabinInflow.Xi_outflow;
  section_1.P_air_gap = cabinInflow.P;
  section_1.X_air_gap = cabinInflow.Xi_outflow;

  L_oct = section_1.R_ext*sqrt(2 - sqrt(2));
  W_fl = L_oct*(1 + sqrt(2));

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
  connect(internalConvection.inlet, cabin.thermalPort)
    annotation (Line(points={{-20,-19.4},{-20,-42},{0,-42}},
                                                           color={191,0,0}));
  connect(cabinInterior.port, internalConvection.outlet)
    annotation (Line(points={{-20,20},{-20,-12.6}}, color={191,0,0}));
  connect(section_5.heatTransmitted, cabinInterior.port)
    annotation (Line(points={{-70,20},{-20,20}}, color={191,0,0}));
  connect(section_1.heatTransmitted, cabinInterior.port)
    annotation (Line(points={{70,20},{-20,20}}, color={191,0,0}));
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
