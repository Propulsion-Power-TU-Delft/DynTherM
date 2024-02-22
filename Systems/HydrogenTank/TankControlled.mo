within DynTherM.Systems.HydrogenTank;
model TankControlled
  outer DynTherM.Components.Environment environment "Environmental properties";

  replaceable package Medium = Media.ExtMedia.CoolProp.Hydrogen constrainedby
    ExternalMedia.Media.BaseClasses.ExternalTwoPhaseMedium "Medium model" annotation(choicesAllMatching = true);

  replaceable model HTC_ext =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    "External convection correlation" annotation (choicesAllMatching=true);

  // Geometry
  parameter Length L_tank "Length of the cylindrical tank section" annotation (Dialog(tab="Geometry"));
  parameter Length R_int "Internal radius of the tank" annotation (Dialog(tab="Geometry"));
  parameter Real AR "Aspect ratio of the end domes" annotation (Dialog(tab="Geometry"));
  parameter Length t_tank "Fuselage/tank thickness" annotation (Dialog(tab="Geometry"));
  parameter Length t_insulation "Insulation thickness" annotation (Dialog(tab="Geometry"));

  parameter Real Ff "Tank fill fraction";
  parameter Modelica.Units.SI.Pressure P_vent "Venting pressure";
  parameter Modelica.Units.SI.MassFlowRate f_vent_max=100
    "Maximum allowable venting mass flow rate";

  // Initialization
  parameter Temperature T_start_tank
    "Fuselage/tank temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start_insulation
    "Insulation temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start
    "Fuel tank pressure - start value" annotation (Dialog(tab="Initialization"));

  // Solar radiation
  parameter Irradiance E_tb[8]
    "Beam component of the clear-sky solar irradiance for each wall section" annotation (Dialog(tab="Solar radiation"));
  parameter Irradiance E_td[8]
    "Diffuse component of the clear-sky solar irradiance for each wall section" annotation (Dialog(tab="Solar radiation"));
  parameter Irradiance E_tr[8]
    "Ground reflected component of the clear-sky solar irradiance for each wall section" annotation (Dialog(tab="Solar radiation"));
  parameter Angle theta[8] "Incidence angle for each wall section" annotation (Dialog(tab="Solar radiation"));

  Tank tank(
    redeclare package Medium = Medium,
    redeclare model HTC_ext = HTC_ext,
    L_tank=L_tank,
    R_int=R_int,
    AR=AR,
    t_tank=t_tank,
    t_insulation=t_insulation,
    Ff=Ff,
    T_start_tank=T_start_tank,
    T_start_insulation=T_start_insulation,
    P_start=P_start,
    E_tb=E_tb,
    E_td=E_td,
    E_tr=E_tr,
    theta=theta)
            annotation (Placement(transformation(extent={{-44,-42},{46,48}})));
  Sensors.MassflowSensorExt m_flow_sensor_LH2(redeclare package Medium = Medium)
  annotation (Placement(transformation(extent={{-100,-48},{-76,-72}})));
  Sensors.MassflowSensorExt m_flow_sensor_VH2(redeclare package Medium = Medium)
  annotation (Placement(transformation(extent={{132,72},{108,48}})));
  Sensors.PressureSensorExt P_sensor_VH2(redeclare package Medium = Medium)
  annotation (Placement(transformation(extent={{108,68},{132,92}})));
  Sensors.PressureSensorExt P_sensor_LH2(redeclare package Medium = Medium)
  annotation (Placement(transformation(extent={{-76,-42},{-100,-18}})));
  Modelica.Blocks.Sources.RealExpression totalTankAndBoilOffMass(
    y=boiledOffHydrogen.y + tank.m_tot) annotation (Placement(transformation(extent={{-18,-80},{2,-60}})));
  Modelica.Blocks.Sources.RealExpression totalHeatToLiquidVapour(
    y=tank.tank_internal.Qwv_tot + tank.tank_internal.Qwl_tot) annotation (Placement(transformation(extent={{-50,-80},{-30,-60}})));
  Modelica.Blocks.Sources.RealExpression boilOffHydrogen(
    y=tank.tank_internal.wev + tank.tank_internal.ws - tank.tank_internal.wc) annotation (Placement(transformation(extent={{12,-80},{32,-60}})));
  Modelica.Blocks.Continuous.Integrator boiledOffHydrogen annotation (Placement(transformation(extent={{42,-80},{62,-60}})));
  Modelica.Blocks.Sources.RealExpression energyLeaving(
    y=tank.tank_internal.ql*tank.tank_internal.hl) annotation (Placement(transformation(extent={{-18,-100},{2,-80}})));
  Modelica.Blocks.Sources.RealExpression theoreticalBoillOff(
    y=totalHeatToLiquidVapour.y/446592) annotation (Placement(transformation(extent={{-50,-100},{-30,-80}})));
  Modelica.Blocks.Sources.RealExpression theoreticalBoillOffFactor(
    y=(theoreticalBoillOff.y + 1e-9)/(boilOffHydrogen.y + 1e-9)) annotation (Placement(transformation(extent={{12,-100},{32,-80}})));
  CustomInterfaces.ExtFluidPort_B extFluidPort_B(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-130,-70},{-110,-50}})));
  CustomInterfaces.ExtFluidPort_B extFluidPort_B1(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{150,50},{170,70}})));
  Modelica.Blocks.Interfaces.RealOutput Pressure_LH2 annotation (Placement(
        transformation(extent={{-116,-42},{-140,-18}}), iconTransformation(
          extent={{-116,-42},{-140,-18}})));
  Modelica.Blocks.Interfaces.RealOutput Pressure_VH2 annotation (Placement(
        transformation(extent={{156,68},{180,92}}),iconTransformation(extent={{-116,8},
            {-140,32}})));
  BoundaryConditions.flow_source_ext VentingMassFlow(
    redeclare package Medium = DynTherM.Media.ExtMedia.CoolProp.Hydrogen,
    T_nom(displayUnit="K") = 20.268,   use_in_massFlow=true)
    "Imposed mass flow rate for venting"
    annotation (Placement(transformation(extent={{12,-12},{-12,12}},
        rotation=-90,
        origin={80,0})));
  Subsystems.TankPressureController pressure_Controller(P_vent=P_vent, f_vent_max=f_vent_max)
                                                         annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={100,-40})));
  Modelica.Blocks.Math.Add add annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={94,-80})));
  Modelica.Blocks.Continuous.Integrator ventedMass annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={100,30})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow RTBC
    annotation (Placement(transformation(extent={{-100,68},{-76,92}})));
  Modelica.Blocks.Interfaces.RealInput Q_RTBC annotation (Placement(
        transformation(extent={{-140,68},{-116,92}}), iconTransformation(
          extent={{-140,68},{-116,92}})));
equation
  connect(boilOffHydrogen.y, boiledOffHydrogen.u)
    annotation (Line(points={{33,-70},{40,-70}}, color={0,0,127}));
  connect(m_flow_sensor_LH2.outlet, tank.LH2_port) annotation (Line(points={{-76,-60},
          {-59.3,-60},{-59.3,-31.2}},         color={0,0,0}));
  connect(P_sensor_LH2.port, tank.LH2_port) annotation (Line(points={{-76,-30},
          {-56,-30},{-56,-31.2},{-59.3,-31.2}},   color={0,0,0}));
  connect(extFluidPort_B, m_flow_sensor_LH2.inlet)
    annotation (Line(points={{-120,-60},{-100,-60}},color={0,0,0}));
  connect(tank.VH2_port, m_flow_sensor_VH2.outlet) annotation (Line(points={{19.9,
          40.8},{19.9,60},{108,60}},  color={0,0,0}));
  connect(tank.VH2_port, P_sensor_VH2.port) annotation (Line(points={{19.9,
          40.8},{20,40.8},{20,80},{108,80}},
                               color={0,0,0}));
  connect(m_flow_sensor_VH2.inlet, extFluidPort_B1)
    annotation (Line(points={{132,60},{160,60}},color={0,0,0}));
  connect(Pressure_LH2, P_sensor_LH2.y)
    annotation (Line(points={{-128,-30},{-101.2,-30}},color={0,0,127}));
  connect(P_sensor_VH2.y, Pressure_VH2)
    annotation (Line(points={{133.2,80},{168,80}},color={0,0,127}));
  connect(VentingMassFlow.outlet, tank.VH2_port)
    annotation (Line(points={{80,12},{80,40.8},{19.9,40.8}}, color={0,0,0}));
  connect(pressure_Controller.Required_Massflow, VentingMassFlow.in_massFlow)
    annotation (Line(points={{100,-30},{100,-9.6},{88.4,-9.6}}, color={0,0,
          127}));
  connect(P_sensor_VH2.y, pressure_Controller.Tank_Pressure) annotation (Line(
        points={{133.2,80},{140,80},{140,-60},{106,-60},{106,-50}}, color={0,
          0,127}));
  connect(add.y, pressure_Controller.External_Massflow)
    annotation (Line(points={{94,-69},{94,-50}}, color={0,0,127}));
  connect(m_flow_sensor_VH2.y, add.u2) annotation (Line(points={{120,46.8},{
          120,-110},{100,-110},{100,-92}}, color={0,0,127}));
  connect(m_flow_sensor_LH2.y, add.u1) annotation (Line(points={{-88,-73.2},{
          -88,-110},{88,-110},{88,-92}}, color={0,0,127}));
  connect(pressure_Controller.Required_Massflow, ventedMass.u)
    annotation (Line(points={{100,-30},{100,18}}, color={0,0,127}));
  connect(Q_RTBC, RTBC.Q_flow)
    annotation (Line(points={{-128,80},{-100,80}}, color={0,0,127}));
  connect(RTBC.port, tank.heat_port) annotation (Line(points={{-76,80},{-60,
          80},{-60,-12},{22,-12},{22,-11.85},{33.85,-11.85}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,
            -120},{160,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{160,
            100}})));
end TankControlled;
