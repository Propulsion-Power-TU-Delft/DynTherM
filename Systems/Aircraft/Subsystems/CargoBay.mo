within DynTherM.Systems.Aircraft.Subsystems;
model CargoBay "Lower section of the fuselage: cargo bay"

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

  parameter HeatFlowRate Q_int "Internal heat load";
  parameter Length R_ext "External radius of the fuselage";
  parameter Length L_cargo "Length of the cargo bay";
  parameter Volume V_cargo "Cargo internal volume";
  parameter Length t_cargo
    "Overall fuselage thickness (cargo section)";

  // Radiation
  parameter Real rho_g=0.2 "Ground reflectance" annotation (Dialog(tab="Radiation"));
  parameter Angle csi[3]={
    Modelica.Units.Conversions.from_deg(135),
    Modelica.Units.Conversions.from_deg(180),
    Modelica.Units.Conversions.from_deg(225)}
    "Tilt angle of the surface wrt horizontal - sections 6-8" annotation (Dialog(tab="Radiation"));
  parameter Angle psi_plus=0 "Modifier of azimuth angle" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_tb[3]
    "Beam component of the clear-sky solar irradiance - sections 6-8" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_td[3]
    "Diffuse component of the clear-sky solar irradiance - sections 6-8" annotation (Dialog(tab="Radiation"));
  parameter Irradiance E_tr[3]
    "Ground reflected component of the clear-sky solar irradiance - sections 6-8" annotation (Dialog(tab="Radiation"));
  parameter Angle theta[3] "Incidence angle - sections 6-8" annotation (Dialog(tab="Radiation"));

  // Initialization
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_fuselage
    "Fuselage temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_cargo
    "Cargo bay temperature start value"
    annotation (Dialog(tab="Initialization"));
  parameter Pressure Pstart_cargo
    "Cargo bay pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature=false "Remove initial equation on temperature" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  Components.MassTransfer.Plenum cargo(
    initOpt=initOpt,
    V=V_cargo,
    Q_int=Q_int,
    P_start=Pstart_cargo,
    T_start=Tstart_cargo,
    noInitialPressure=noInitialPressure,
    noInitialTemperature=noInitialTemperature,
    fixed_Q=false)
    annotation (Placement(transformation(extent={{-22,42},{22,-2}})));
  CustomInterfaces.FluidPort_A cargoToCabin annotation (Placement(
        transformation(extent={{-106,14},{-94,26}}), iconTransformation(extent={{50,70},
            {70,90}})));
  CustomInterfaces.FluidPort_B cargoOutflow annotation (Placement(
        transformation(extent={{94,14},{106,26}}),     iconTransformation(
          extent={{-10,-110},{10,-90}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b cargoToFloor
    annotation (Placement(transformation(extent={{-10,90},{10,110}}),
        iconTransformation(extent={{-70,70},{-50,90}})));
  Systems.Aircraft.Subsystems.LowerFuselageHeatTransfer section_6(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cargo,
    R_ext=R_ext,
    t_fuselage=t_cargo,
    rho_g=rho_g,
    csi=csi[1],
    psi_plus=psi_plus,
    E_tb=E_tb[1],
    E_td=E_td[1],
    E_tr=E_tr[1],
    theta=theta[1],
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-80,-56},{-40,-96}})));
  Systems.Aircraft.Subsystems.LowerFuselageHeatTransfer section_7(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cargo,
    R_ext=R_ext,
    t_fuselage=t_cargo,
    rho_g=rho_g,
    csi=csi[2],
    psi_plus=psi_plus,
    E_tb=E_tb[2],
    E_td=E_td[2],
    E_tr=E_tr[2],
    theta=theta[2],
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-20,-56},{20,-96}})));
  Systems.Aircraft.Subsystems.LowerFuselageHeatTransfer section_8(
    redeclare model Paint = Paint,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    initOpt=initOpt,
    coeff=1/8,
    L_fuselage=L_cargo,
    R_ext=R_ext,
    t_fuselage=t_cargo,
    rho_g=rho_g,
    csi=csi[3],
    psi_plus=psi_plus,
    E_tb=E_tb[3],
    E_td=E_td[3],
    E_tr=E_tr[3],
    theta=theta[3],
    Tstart_fuselage=Tstart_fuselage)
    annotation (Placement(transformation(extent={{40,-56},{80,-96}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{66,70},{86,90}})));
  Modelica.Blocks.Interfaces.RealOutput cargoTemperature annotation (
      Placement(transformation(extent={{94,68},{118,92}}),
        iconTransformation(extent={{88,-2},{112,22}})));
  Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{66,38},{90,62}})));
  Modelica.Blocks.Interfaces.RealOutput cargoPressure annotation (Placement(
        transformation(extent={{94,38},{118,62}}),   iconTransformation(
          extent={{88,-42},{112,-18}})));
equation
  connect(cargoOutflow,cargo. outlet)
    annotation (Line(points={{100,20},{22,20}},               color={0,0,0}));
  connect(cargo.inlet, cargoToCabin)
    annotation (Line(points={{-22,20},{-100,20}},          color={0,0,0}));
  connect(cargoToFloor, cargo.thermalPort)
    annotation (Line(points={{0,100},{0,0.2}},  color={191,0,0}));
  connect(section_6.heatToInner, cargo.thermalPort)
    annotation (Line(points={{-60,-64},{-60,0.2},{0,0.2}}, color={191,0,0}));
  connect(section_7.heatToInner, cargo.thermalPort)
    annotation (Line(points={{0,-64},{0,0.2},{0,0.2}}, color={191,0,0}));
  connect(section_8.heatToInner, cargo.thermalPort)
    annotation (Line(points={{60,-64},{60,0.2},{0,0.2}}, color={191,0,0}));
  connect(temperatureSensor.T,cargoTemperature)
    annotation (Line(points={{87,80},{106,80}},   color={0,0,127}));
  connect(pressureSensor.y,cargoPressure)
    annotation (Line(points={{91.2,50},{106,50}},   color={0,0,127}));
  connect(cargo.thermalPort, temperatureSensor.port)
    annotation (Line(points={{0,0.2},{0,80},{66,80}}, color={191,0,0}));
  connect(cargo.inlet, pressureSensor.port)
    annotation (Line(points={{-22,20},{-22,50},{66,50}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          lineThickness=0.5),
        Line(
          points={{-98,-10},{98,-10}},
          color={0,0,0},
          thickness=0.5),
        Polygon(points={{-80,-20},{80,-20},{80,-40},{40,-80},{-40,-80},{-80,-40},
              {-80,-20}}, lineColor={0,0,0}),
        Line(
          points={{-88,10},{-94,-14},{-86,-38},{-62,-66},{-34,-46}},
          color={238,46,47},
          smooth=Smooth.Bezier),
        Line(
          points={{88,10},{94,-14},{86,-38},{62,-66},{34,-46}},
          color={238,46,47},
          smooth=Smooth.Bezier),
        Line(
          points={{-34,-46},{-38,-56}},
          color={238,46,47},
          smooth=Smooth.Bezier),
        Line(
          points={{-34,-46},{-44,-46}},
          color={238,46,47},
          smooth=Smooth.Bezier),
        Line(
          points={{44,-46},{34,-46}},
          color={238,46,47},
          smooth=Smooth.Bezier),
        Line(
          points={{34,-46},{38,-56}},
          color={238,46,47},
          smooth=Smooth.Bezier)}),                               Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CargoBay;
