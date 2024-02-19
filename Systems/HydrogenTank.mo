within DynTherM.Systems;
package HydrogenTank

  model Tank
    "Tank of liquid hydrogen embedded in the fuselage (not saturated)"
    // Hp: the fuselage is modelled as a hollow cylinder
    // Hp: the fuel tank is modelled as a hollow cylinder
    // Hp: the heat transfer through the aft pressure bulkhead is neglected

    replaceable package Medium = Media.ExtMedia.CoolProp.Hydrogen constrainedby
      ExternalMedia.Media.BaseClasses.ExternalTwoPhaseMedium "Medium model" annotation(choicesAllMatching = true);

    replaceable model HTC_ext =
      DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
      constrainedby
      DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
      "External convection correlation" annotation (choicesAllMatching=true);

    parameter Boolean allowFlowReversal
      "= true to allow flow reversal, false restricts to design direction" annotation(Evaluate=true);

    // Geometry
    parameter Length L_tank "Length of the cylindrical tank section" annotation (Dialog(tab="Geometry"));
    parameter Length R_int "Internal radius of the tank" annotation (Dialog(tab="Geometry"));
    parameter Real AR "Aspect ratio of the end domes" annotation (Dialog(tab="Geometry"));
    parameter Length t_tank "Fuselage/tank thickness" annotation (Dialog(tab="Geometry"));
    parameter Length t_insulation "Insulation thickness" annotation (Dialog(tab="Geometry"));
    final parameter Mass m_tot=
      tank_section_1.m + tank_section_2.m + tank_section_3.m + tank_section_4.m +
      tank_section_5.m + tank_section_6.m + tank_section_7.m + tank_section_8.m
      "Mass of the total fuselage/tank + insulation structure";

    parameter Real Ff "Tank fill fraction";
    constant Real pi=Modelica.Constants.pi;

    // Initialization
    parameter Choices.InitOpt initOpt "Initialisation option" annotation (Dialog(tab="Initialization"));
    parameter Boolean noInitialPressure=false
      "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
    parameter Temperature T_start_tank
      "Fuselage/tank temperature - start value" annotation (Dialog(tab="Initialization"));
    parameter Temperature T_start_insulation
      "Insulation temperature - start value" annotation (Dialog(tab="Initialization"));
    parameter Pressure P_start
      "Fuel tank pressure - start value" annotation (Dialog(tab="Initialization"));
    parameter Volume Vl_start=130.2155 + (tank_internal.Vt - 155.73479513776527)/68.4
      "Fuel volume - start value (120.76)" annotation (Dialog(tab="Initialization"));

    // Solar radiation
    parameter Irradiance E_tb[8]
      "Beam component of the clear-sky solar irradiance for each wall section" annotation (Dialog(tab="Solar radiation"));
    parameter Irradiance E_td[8]
      "Diffuse component of the clear-sky solar irradiance for each wall section" annotation (Dialog(tab="Solar radiation"));
    parameter Irradiance E_tr[8]
      "Ground reflected component of the clear-sky solar irradiance for each wall section" annotation (Dialog(tab="Solar radiation"));
    parameter Angle theta[8] "Incidence angle for each wall section" annotation (Dialog(tab="Solar radiation"));

    Subsystems.TankInternal tank_internal(
      redeclare package Medium = Medium,
      allowFlowReversal=allowFlowReversal,
      Ff=Ff,
      initOpt=initOpt,
      noInitialPressure=noInitialPressure,
      P_start=P_start,
      Vl_start=Vl_start,
      L=L_tank,
      R_int=R_int,
      AR=AR) annotation (Placement(transformation(extent={{-38,-46},{52,44}})));
    Subsystems.CylinderHeatTransfer tank_section_1(
      redeclare model HTC_ext = HTC_ext,
      coeff=1/8,
      L_tank=L_tank,
      R_ext=R_int + t_tank + t_insulation,
      t_tank=t_tank,
      t_insulation=t_insulation,
      csi=3.1415926535898,
      E_tb=E_tb[1],
      E_td=E_td[1],
      E_tr=E_tr[1],
      theta=theta[1],
      T_start_tank=T_start_tank,
      T_start_insulation=T_start_insulation)
      annotation (Placement(transformation(extent={{-80,52},{-40,92}})));
    Subsystems.CylinderHeatTransfer tank_section_2(
      redeclare model HTC_ext = HTC_ext,
      coeff=1/8,
      L_tank=L_tank,
      R_ext=R_int + t_tank + t_insulation,
      t_tank=t_tank,
      t_insulation=t_insulation,
      csi=2.3561944901923,
      E_tb=E_tb[2],
      E_td=E_td[2],
      E_tr=E_tr[2],
      theta=theta[2],
      T_start_tank=T_start_tank,
      T_start_insulation=T_start_insulation)
      annotation (Placement(transformation(extent={{-40,52},{0,92}})));
    Subsystems.CylinderHeatTransfer tank_section_3(
      redeclare model HTC_ext = HTC_ext,
      coeff=1/8,
      L_tank=L_tank,
      R_ext=R_int + t_tank + t_insulation,
      t_tank=t_tank,
      t_insulation=t_insulation,
      csi=1.5707963267949,
      E_tb=E_tb[3],
      E_td=E_td[3],
      E_tr=E_tr[3],
      theta=theta[3],
      T_start_tank=T_start_tank,
      T_start_insulation=T_start_insulation)
      annotation (Placement(transformation(extent={{0,52},{40,92}})));
    Subsystems.CylinderHeatTransfer tank_section_4(
      redeclare model HTC_ext = HTC_ext,
      coeff=1/8,
      L_tank=L_tank,
      R_ext=R_int + t_tank + t_insulation,
      t_tank=t_tank,
      t_insulation=t_insulation,
      csi=0.78539816339745,
      E_tb=E_tb[4],
      E_td=E_td[4],
      E_tr=E_tr[4],
      theta=theta[4],
      T_start_tank=T_start_tank,
      T_start_insulation=T_start_insulation)
      annotation (Placement(transformation(extent={{40,52},{80,92}})));
    Subsystems.CylinderHeatTransfer tank_section_5(
      redeclare model HTC_ext = HTC_ext,
      coeff=1/8,
      L_tank=L_tank,
      R_ext=R_int + t_tank + t_insulation,
      t_tank=t_tank,
      t_insulation=t_insulation,
      csi=0,
      E_tb=E_tb[5],
      E_td=E_td[5],
      E_tr=E_tr[5],
      theta=theta[5],
      T_start_tank=T_start_tank,
      T_start_insulation=T_start_insulation)
      annotation (Placement(transformation(extent={{-80,-52},{-40,-92}})));
    Subsystems.CylinderHeatTransfer tank_section_6(
      redeclare model HTC_ext = HTC_ext,
      coeff=1/8,
      L_tank=L_tank,
      R_ext=R_int + t_tank + t_insulation,
      t_tank=t_tank,
      t_insulation=t_insulation,
      csi=5.4977871437821,
      E_tb=E_tb[6],
      E_td=E_td[6],
      E_tr=E_tr[6],
      theta=theta[6],
      T_start_tank=T_start_tank,
      T_start_insulation=T_start_insulation)
      annotation (Placement(transformation(extent={{-40,-52},{0,-92}})));
    Subsystems.CylinderHeatTransfer tank_section_7(
      redeclare model HTC_ext = HTC_ext,
      coeff=1/8,
      L_tank=L_tank,
      R_ext=R_int + t_tank + t_insulation,
      t_tank=t_tank,
      t_insulation=t_insulation,
      csi=4.7123889803847,
      E_tb=E_tb[7],
      E_td=E_td[7],
      E_tr=E_tr[7],
      theta=theta[7],
      T_start_tank=T_start_tank,
      T_start_insulation=T_start_insulation)
      annotation (Placement(transformation(extent={{0,-52},{40,-92}})));
    Subsystems.CylinderHeatTransfer tank_section_8(
      redeclare model HTC_ext = HTC_ext,
      coeff=1/8,
      L_tank=L_tank,
      R_ext=R_int + t_tank + t_insulation,
      t_tank=t_tank,
      t_insulation=t_insulation,
      csi=3.9269908169872,
      E_tb=E_tb[8],
      E_td=E_td[8],
      E_tr=E_tr[8],
      theta=theta[8],
      T_start_tank=T_start_tank,
      T_start_insulation=T_start_insulation)
      annotation (Placement(transformation(extent={{40,-52},{80,-92}})));
    Subsystems.DomeHeatTransfer dome_2(
      redeclare model HTC_ext = HTC_ext,
      coeff=0.5,
      L_tank=L_tank,
      R_ext=R_int + t_tank + t_insulation,
      t_tank=t_tank,
      t_insulation=t_insulation,
      AR=AR,
      csi=1.5707963267949,
      E_tb=E_tb[2],
      E_td=E_td[2],
      E_tr=E_tr[2],
      theta=theta[2],
      T_start_tank=T_start_tank,
      T_start_insulation=T_start_insulation)
      annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=-90,
          origin={84,0})));
    Subsystems.DomeHeatTransfer dome_1(
      redeclare model HTC_ext = HTC_ext,
      coeff=0.5,
      L_tank=L_tank,
      R_ext=R_int + t_tank + t_insulation,
      t_tank=t_tank,
      t_insulation=t_insulation,
      AR=AR,
      csi=1.5707963267949,
      E_tb=E_tb[2],
      E_td=E_td[2],
      E_tr=E_tr[2],
      theta=theta[2],
      T_start_tank=T_start_tank,
      T_start_insulation=T_start_insulation)
      annotation (Placement(transformation(
          extent={{-20,20},{20,-20}},
          rotation=-90,
          origin={-84,0})));
    CustomInterfaces.ExtFluidPort_A LH2_port(redeclare package Medium = Medium) annotation (Placement(transformation(
            extent={{-108,-42},{-92,-26}}), iconTransformation(extent={{-142,-84},
              {-126,-68}})));
    CustomInterfaces.ExtFluidPort_A VH2_port(redeclare package Medium = Medium) annotation (Placement(transformation(
            extent={{92,32},{108,48}}), iconTransformation(extent={{34,76},{50,92}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heat_port annotation (
        Placement(transformation(extent={{90,-50},{110,-30}},rotation=0),
          iconTransformation(extent={{64,-42},{82,-24}})));

  equation
    // wall connections
    connect(tank_internal.wall.ports[1,1], tank_section_1.heatToInner);
    connect(tank_internal.wall.ports[2,1], tank_section_2.heatToInner);
    connect(tank_internal.wall.ports[3,1], tank_section_3.heatToInner);
    connect(tank_internal.wall.ports[4,1], tank_section_4.heatToInner);
    connect(tank_internal.wall.ports[5,1], tank_section_5.heatToInner);
    connect(tank_internal.wall.ports[6,1], tank_section_6.heatToInner);
    connect(tank_internal.wall.ports[7,1], tank_section_7.heatToInner);
    connect(tank_internal.wall.ports[8,1], tank_section_8.heatToInner);

    connect(LH2_port, tank_internal.LH2_outlet) annotation (Line(points={{-100,-34},
            {-57.35,-34},{-57.35,-32.95}}, color={0,0,0}));
    connect(VH2_port, tank_internal.VH2_outlet) annotation (Line(points={{100,40},
            {20.95,40},{20.95,39.05}}, color={0,0,0}));
    connect(tank_internal.heat_port, heat_port) annotation (Line(points={{32.65,-16.75},
            {32.65,-40},{100,-40}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Bitmap(extent={{-210,-142},{234,146}}, fileName="modelica://DynTherM/Figures/LH2Tank.png")}),
                                                                   Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Tank;

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

  package Subsystems
    model TankInternal "Internal part of the tank (no heat transfer through walls)"

      replaceable package Medium = Media.ExtMedia.CoolProp.Hydrogen constrainedby
        ExternalMedia.Media.BaseClasses.ExternalTwoPhaseMedium "Medium model" annotation(choicesAllMatching = true);

      // Options and constants
      parameter Boolean allowFlowReversal
        "= true to allow flow reversal, false restricts to design direction" annotation(Evaluate=true);
      parameter Real Ff "Tank Fill fraction";
      parameter Time tauev = 1e1 "Time constant of bulk evaporation";
      parameter Time tauc = 1e1 "Time constant of bulk condensation";

      //parameter Real Kcs = 0 "Surface condensation coefficient [kg/(s.m2.K)]";
      //parameter Real Kes = 0 "Surface evaporation coefficient [kg/(s.m2.K)]";
      //parameter Real Kvs = 0 "Surface heat transfer coefficient (vapor-surface) [W/(m2.K)]";
      //parameter Real Kls = 0 "Surface heat transfer coefficient (liquid-surface) [W/(m2.K)]";

      constant Acceleration g=Modelica.Constants.g_n;
      constant Real pi=Modelica.Constants.pi;

      // Initialization
      parameter Choices.InitOpt initOpt "Initialisation option" annotation (Dialog(tab="Initialization"));
      parameter Boolean noInitialPressure=false
        "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
      parameter Pressure P_start "Pressure - start value" annotation (Dialog(tab="Initialization"));
      parameter Medium.SpecificEnthalpy hl_start=Medium.bubbleEnthalpy(Medium.setSat_p(P_start))
        "Enthalpy of liquid hydrogen - start value" annotation (Dialog(tab="Initialization"));
      parameter Medium.SpecificEnthalpy hv_start=Medium.dewEnthalpy(Medium.setSat_p(P_start))
        "Enthalpy of gaseous hydrogen - start value" annotation (Dialog(tab="Initialization"));
      parameter Length y_start = 0.808*Ff*R_int
        "Liquid level referred to the centreline - start value" annotation (Dialog(tab="Initialization"));
      parameter Volume Vl_start = Vt*Ff "Liquid fluid volume";

      // Geometry
      parameter Length L "Length" annotation (Dialog(tab="Geometry"));
      parameter Length R_int "Internal radius" annotation (Dialog(tab="Geometry"));
      parameter Real AR = 1 "Tank end-dome aspect ratio (h/R)" annotation (Dialog(tab="Geometry"));

      final parameter Length R_eq = R_int*(1 + AR^2)/(2*AR)
        "Radius of equivalent sphere when dome AR < 1";
      final parameter Length L_star = R_eq - AR*R_int
        "Length of aproximated cylinder which is cut of from a half sphere when dome AR < 1";
      final parameter Length L_star_loc = L_star/2
        "Horizontal location of R_star (middle of the cylinder)";
      final parameter Length R_star = R_eq*cos(asin(L_star_loc/R_eq))
        "Radius of aproximated cylinder which is cut of from a half sphere when dome AR < 1";
      final parameter Integer n_star = 1 "Number of equivalent sections of volume cylinders";
      final parameter Length dR = R_eq - R_int
        "Difference between radius of dome and equivalents sphere";
      final parameter Volume Vt = pi*R_int^2*L +
        2*pi*(R_int*AR)^2*(R_eq - (R_int*AR)/3) "Internal volume";
      final parameter Area Awt_tot = 2*pi*R_int*L + 2*pi*R_eq^2*AR "Internal area";

      Real A_frac "Liquid area fraction";
      Length y(start = y_start) "Liquid level referred to the centreline";
      Length y0 "Liquid level referred to the bottom";
      Length dh_outer "Height of wall segments 0 and 4 (top and bottom)";
      Length dh_between "Height of wall segments 1, 3, 5 and 7";
      Length dh_inner "Height of wall segments 2 and 6 (central sections)";

      // Thermodynamics States
      Medium.SaturationProperties sat "Saturated state";
      Medium.ThermodynamicState liq "Thermodynamic state of the liquid";
      Medium.ThermodynamicState vap "Thermodynamic state of the vapor";

      // Mass flow rates
      MassFlowRate ql "Liquid mass flow rate";
      MassFlowRate qv "Gaseous mass flow rate";
      MassFlowRate wc "Mass flow rate of bulk condensation";
      MassFlowRate wev "Mass flow rate of bulk evaporation";
      MassFlowRate ws "Mass flow rate of surface evaporation/condensation
    (positive for evaporation, negative for condensation)";
      //MassFlowRate wcs "Mass flow rate of surface condensation";
      //MassFlowRate wes "Mass flow rate of surface evaporation";

      // Fluid Properties
      Mass Mv "Mass of vapor";
      Mass Ml "Mass of liquid";
      Volume Vv(start = Vt - Vl_start) "Volume of vapor";
      Volume Vl(start = Vl_start, stateSelect=StateSelect.default) "Volume of liquid";
      Energy El "Liquid internal energy";
      Energy Ev "Vapour internal energy";
      PerUnit xl "Mass fraction of vapor in the liquid volume";
      PerUnit xv "Steam quality in the vapor volume";
      Medium.AbsolutePressure P(start=P_start, stateSelect=StateSelect.prefer)
        "Surface pressure";
      SpecificEnthalpy hls "Specific enthalpy of saturated liquid";
      SpecificEnthalpy hvs "Specific enthalpy of saturated vapor";
      SpecificEnthalpy Dh "Latent Heat of Evaporation at boiling point";
      Medium.SpecificEnthalpy hl(start=hl_start, stateSelect=StateSelect.prefer)
        "Specific enthalpy of liquid";
      Medium.SpecificEnthalpy hv(start=hv_start, stateSelect=StateSelect.prefer)
        "Specific enthalpy of vapor";
      Medium.Temperature Ts "Saturation temperature";
      Medium.Temperature Tl "Liquid temperature";
      Medium.Temperature Tv "Vapour temperature";
      Density rhol "Liquid density";
      Density rhov "Vapour density";
      Medium.SpecificInternalEnergy ul "Liquid specific internal energy";
      Medium.SpecificInternalEnergy uv "Vapour specific internal energy";

      // Wall temperatures
      Temperature Tw[8] "Wall temperature";
      Temperature Tw_dome[2] "Wall temperature of the dome";

      // Heat flow rates
      Power Q_tot "Total heat flow from wall to inner";
      Power Qwv_tot "Heat flow from the wall to the vapor";
      Power Qwl_tot "Heat flow from the wall to the liquid";
      Power Ql_tot "Total heat flow to the liquid";
      Power Qv_tot "Total heat flow to the vapor";
      Power Qwv[8] "Heat flow from the wall to the vapor";
      Power Qwl[8] "Heat flow from the wall to the liquid";
      Power Qwv_dome[2] "Heat flow from the wall to the dome";
      Power Qwl_dome[2] "Heat flow from the wall to the dome";
      Power Qsv "Heat flow from the vapor to the liquid";
      Power Qsl "Heat flow from the liquid to the vapor";
      Power Ql_int "Heat flow introduced into the liquid directly";
      Power Qv_int "Heat flow introduced into the vapor directly";
      Power Q_int "Heat flow introduced into the fluid (liq. and vap.) directly";

      // Wall Interface Surface Areas
      Area Awl_tot "Surface of the wall-liquid interface";
      Area Awv_tot "Surface of the wall-vapor interface";
      Area Awl[8] "Surface of the wall-liquid interface";
      Area Awv[8] "Surface of the wall-vapor interface";
      Area Awl_dome[2] "Surface of the wall-liquid interface at the dome";
      Area Awv_dome[2] "Surface of the wall-vapor interface at the dome";
      Area Asup "Surface of the liquid-vapor interface";
      Area Awt[8] "Surface of the wall";
      Area Awt_dome[2] "Surface of the dome";

      // Heat Transfer Coefficients and Non-Dimensional Numbers
      PrandtlNumber Prl "Prandtl number of the liquid";
      PrandtlNumber Prv "Prandtl number of the vapor";

      NusseltNumber Nul[8] "Nusselt number of the liquid";
      NusseltNumber Nuv[8] "Nusselt number of the vapor";
      NusseltNumber Nul_dome[2] "Nusselt number of the liquid at the dome";
      NusseltNumber Nuv_dome[2] "Nusselt number of the vapor at the dome";

      RayleighNumber Ral[8] "Rayleigh number of the liquid";
      RayleighNumber Rav[8] "Rayleigh number of the vapor";
      RayleighNumber Ral_dome[2] "Rayleigh number of the liquid at the dome";
      RayleighNumber Rav_dome[2] "Rayleigh number of the vapor at wall at the dome";

      CoefficientOfHeatTransfer h_int_l[8] "Internal convective heat transfer of the liquid";
      CoefficientOfHeatTransfer h_int_v[8] "Internal convective heat transfer of the vapor";
      CoefficientOfHeatTransfer h_int_l_dome[2] "Internal convective heat transfer of the liquid at the dome";
      CoefficientOfHeatTransfer h_int_v_dome[2] "Internal convective heat transfer of the vapor at the dome";

      CoefficientOfHeatTransfer h_rad_l[8] "Internal radiation equivalent convective coefficient for the liquid";
      CoefficientOfHeatTransfer h_rad_v[8] "Internal radiation equivalent convective coefficient for the vapor";
      CoefficientOfHeatTransfer h_rad_l_dome[2] "Internal radiation equivalent convective coefficient for the liquid at the dome";
      CoefficientOfHeatTransfer h_rad_v_dome[2] "Internal radiation equivalent convective coefficient for the vapor at the dome";

      CoefficientOfHeatTransfer h_tot_l[8] "Total internal heat transfer coefficient for the liquid";
      CoefficientOfHeatTransfer h_tot_v[8] "Total internal heat transfer of the vapor";
      CoefficientOfHeatTransfer h_tot_l_dome[2] "Total internal heat transfer coefficient for the liquid at the dome";
      CoefficientOfHeatTransfer h_tot_v_dome[2] "Total internal heat transfer of the vapor at the dome";

        CustomInterfaces.ExtFluidPort_B LH2_outlet(redeclare package Medium = Medium,
          m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0)) annotation (Placement(transformation(extent={{-154,
                -82},{-132,-60}},
                  rotation=0), iconTransformation(extent={{-154,-82},{-132,-60}})));
        CustomInterfaces.ExtFluidPort_B VH2_outlet(redeclare package Medium = Medium,
          m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0)) annotation (Placement(transformation(extent={{22,80},
                {40,98}},                                                                                                                      rotation=0),
                iconTransformation(extent={{22,80},{40,98}})));
        CustomInterfaces.DistributedHeatPort_A wall(Nx=8, Ny=1) annotation (Placement(transformation(extent={{-48,42},
                {-8,82}},            rotation=0), iconTransformation(extent={{-20,-20},
                {20,20}},
            rotation=0,
            origin={-28,62})));
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heat_port annotation (
          Placement(transformation(extent={{10,-72},{48,-34}}, rotation=0),
            iconTransformation(extent={{48,-44},{66,-26}})));
        CustomInterfaces.DistributedHeatPort_A dome(Nx=2, Ny=1) annotation (Placement(transformation(extent={{-116,66},{-90,92}},  rotation=0), iconTransformation(extent={{-22,-22},
                {22,22}},
            rotation=90,
            origin={104,-40})));

    equation
      // Conservation equations
      der(Mv) = qv + wev - wc + ws "Vapour volume mass balance";
      der(Ml) = ql - wev + wc - ws "Liquid volume mass balance";
      der(Ev) = qv*hv + wev*hvs - wc*hls + ws*hvs + Qv_tot - Qsv - P*der(Vv) "Vapour volume energy balance";
      der(El) = ql*hl - wev*hvs + wc*hls - ws*hls + Ql_tot + Qsl - P*der(Vl) "Liquid volume energy balance";
      //(Ev) = qv*hv + (wev - wcs)*hvs - wc*hls + Qv_tot - Qvl - p*der(Vv) "Vapour volume energy balance";
      //der(El) = ql*hl + (wcs - wev)*hvs + wc*hls + Ql_tot + Qvl - p*der(Vl) "Liquid volume energy balance";

      // Fluid properties
      sat = Medium.setSat_p(P);
      liq = Medium.setState_ph(P, hl, 2);
      Tl = Medium.temperature_ph(P, hl, 1);
      rhol = Medium.density_ph(P, hl, 1);
      ul = Medium.specificInternalEnergy(liq);
      vap = Medium.setState_ph(P, hv, 2);
      Tv = Medium.temperature_ph(P, hv, 1);
      rhov = Medium.density_ph(P, hv, 1);
      uv = Medium.specificInternalEnergy(vap);

      hls = sat.hl;
      hvs = sat.hv;
      Ts = sat.Tsat;
      Dh = hvs - hls;

      Mv = Vv*rhov;
      Ml = Vl*rhol;
      Ev = Mv*(hv - P/rhov); //Medium.specificInternalEnergy(vap);
      El = Ml*(hl - P/rhol); //Medium.specificInternalEnergy(liq);

      wev = xl*rhol*Vl/tauev;
      wc = (1 - xv)*rhov*Vv/tauc;
      //wcs = Kcs*Asup*(Ts - Tl);
      //wes = Kes*Asup*(Ts - Tv);
      ws = (Qsv - Qsl)/Dh;
      //wes = homotopy(if Qsv >= Qsl then ws else 0, 0);
      //wcs = homotopy(if Qsl >= Qsv then -ws else 0, 0);

      //Qvl = Ks*Asup*(Tv - Ts);
      Qsv = 2*Asup*vap.lambda*(Tv - Ts)/((R_int - y)/2);
      Qsl = 2*Asup*liq.lambda*(Ts - Tl)/((R_int + y)/2);
      Q_tot = sum(Qwl) + sum(Qwv) + Q_int;

      xv = homotopy(if hv >= hvs then 1 else (hv - hls)/(hvs - hls),
        (hv - hls)/(hvs - hls));
      xl = homotopy(if hl <= hls then 0 else (hl - hls)/(hvs - hls), 0);

      Prl = (liq.eta*liq.cp)/(liq.lambda);
      Prv = (vap.eta*vap.cp)/(vap.lambda);

      y0 = y + R_int "Level (referred to the bottom)";
      Vl = L*(R_int^2*acos(-y/R_int) + y*sqrt(R_int^2 - y^2)) +
        pi*(y0 + dR)^2*(R_eq - 1/3*(y0 + dR)) -
        2*(L_star*(R_star^2*acos(-y/R_star) + y*sqrt(R_star^2 - y^2)));
      Awl_tot = 2*R_int*acos(-y/R_int)*L + 2*pi*R_eq*(y0 + dR) -
        2*(2*R_star*acos(-y/R_star)*L_star);
      //Awl = 2*R_int*acos(-y/R_int)*L + 2*pi*R_eq*(y0 + dR) - 2* (2*pi*R_eq*(R_eq - AR*R_int)*((y0 + dR)/R_eq));
      Asup = 2*sqrt(R_int^2 - y^2)*L + pi*(R_eq*cos(asin((y)/R_eq)))^2 - 2* (2*sqrt(R_star^2 - y^2)*L_star);

      Vt = Vl + Vv;
      Awt_tot = Awv_tot + Awl_tot;
      //Awv_tot = 2*pi*R_int*L - Awl "Metal-vapor interface area";

      // Define difference in height between the separation lines that divide the different wall segments
      dh_outer = R_int*(1-cos(Modelica.Units.Conversions.from_deg(22.5)))
        "Height of wall segments 0 and 4 (top and bottom)";
      dh_between = R_int*(1-cos(Modelica.Units.Conversions.from_deg(22.5+45))) - dh_outer
        "Height of wall segments 1,3,5, and 7";
      dh_inner = (R_int - dh_outer - dh_between)*2
        "Height of wall segments 2 and 6 (central sections)";

      // Calculate the surface area available for heat transfer on the inner tank, for each wall segment
      Awt[1] = 2*R_int*acos(-(-(dh_inner/2+dh_between))/R_int)*L;
      Awt[2] = (2*R_int*acos(-(-(dh_inner/2))/R_int)*L - Awt[1])/2;
      Awt[3] = (2*R_int*acos(-(dh_inner/2)/R_int)*L - Awt[1] - 2*Awt[2])/2;
      Awt[4] = Awt[2];
      Awt[5] = Awt[1];
      Awt[6] = Awt[2];
      Awt[7] = Awt[3];
      Awt[8] = Awt[2];
      //Awt_dome[1] = 0.5 * 4*pi*R_int^2*((1+2*AR^1.6075)/3)^(1/1.6075) "Inner wall surface area of dome 1";
      Awt_dome[1] = 2*pi*R_eq*(R_int*AR);
      Awt_dome[2] = Awt_dome[1];

      // Calculate surface area in contact with the liquid for each wall segment
      if y >= (dh_inner/2 + dh_between) then
        Awl[1] = Awt[1];
        Awl[2] = Awt[2];
        Awl[3] = Awt[3];
        Awl[4] = Awt[4];
        Awl[5] = 2*R_int*acos(-y/R_int)*L - Awt[8] - Awt[7] - Awt[6] - Awt[4] - Awt[3] - Awt[2] - Awt[1];
        Awl[6] = Awt[6];
        Awl[7] = Awt[7];
        Awl[8] = Awt[8];

      elseif y >= (dh_inner/2) then
        Awl[1] = Awt[1];
        Awl[2] = Awt[2];
        Awl[3] = Awt[3];
        Awl[4] = (2*R_int*acos(-y/R_int)*L - Awt[8] - Awt[7] - Awt[3] - Awt[2] - Awt[1])/2;
        Awl[5] = 0;
        Awl[6] = (2*R_int*acos(-y/R_int)*L - Awt[8] - Awt[7] - Awt[3] - Awt[2] - Awt[1])/2;
        Awl[7] = Awt[7];
        Awl[8] = Awt[8];

      elseif y >= -(dh_inner/2) then
        Awl[1] = Awt[1];
        Awl[2] = Awt[2];
        Awl[3] = (2*R_int*acos(-y/R_int)*L - Awt[8] - Awt[2] - Awt[1])/2;
        Awl[4] = 0;
        Awl[5] = 0;
        Awl[6] = 0;
        Awl[7] = (2*R_int*acos(-y/R_int)*L - Awt[8] - Awt[2] - Awt[1])/2;
        Awl[8] = Awt[8];

      elseif y >= -(dh_inner/2 + dh_between) then
        Awl[1] = Awt[1];
        Awl[2] = (2*R_int*acos(-y/R_int)*L - Awt[1])/2;
        Awl[3] = 0;
        Awl[4] = 0;
        Awl[5] = 0;
        Awl[6] = 0;
        Awl[7] = 0;
        Awl[8] = (2*R_int*acos(-y/R_int)*L - Awt[1])/2;

      else
        Awl[1] = 2*R_int*acos(-y/R_int)*L;
        Awl[2] = 0;
        Awl[3] = 0;
        Awl[4] = 0;
        Awl[5] = 0;
        Awl[6] = 0;
        Awl[7] = 0;
        Awl[8] = 0;
      end if;

      for i in 1:8 loop
        // Calculate surface area in contact with the vapor for each wall segment
        Awv[i] = Awt[i] - Awl[i];

        // Calculate Rayleigh number
        Ral[i] = (g*0.01658*abs(Tw[i] - Tl)*((R_int + y)^3)*Prl*rhol^2)/(liq.eta^2);
        Rav[i] = (g*0.01658*abs(Tw[i] - Tv)*((R_int - y)^3)*Prv*rhov^2)/(vap.eta^2);

        // Calculate Nusselt number
        Nul[i] = 0.0605*(Ral[i]^(1/3));
        Nuv[i] = 0.364*2*R_int/(R_int - y)*(Rav[i]^(1/4));

        // Calculate internal convective heat transfer coefficient
        h_int_l[i] = (Nul[i]*liq.lambda)/(R_int + y);
        h_int_v[i] = (Nuv[i]*vap.lambda)/(R_int - y);

        // Calculate equivalent radiative heat transfer coefficient
        h_rad_l[i] = Modelica.Constants.sigma*0.09*(Tw[i]^2 + Tl^2)*(Tw[i] + Tl);
        h_rad_v[i] = 0;  // Assumption

        // Calculate total heat transfer coefficient
        h_tot_l[i] = h_int_l[i] + h_rad_l[i];
        h_tot_v[i] = h_int_v[i] + h_rad_v[i];

        // Heat flows of the wall segments
        Qwl[i] = h_tot_l[i]*Awl[i]*(Tw[i] - Tl);
        Qwv[i] = h_tot_v[i]*Awv[i]*(Tw[i] - Tv);

        // Boundary conditions
        wall.ports[i,1].T = Tw[i];
        Qwv[i] + Qwl[i] = wall.ports[i,1].Q_flow;
      end for;

      for i in 1:2 loop
        // Calculate surface area in contact with the vapor at the domes
        Awv_dome[i] = Awt_dome[i] - Awl_dome[i];

        // Calculate surface area in contact with the liquid at the domes
        Awl_dome[i] = pi*R_eq*(y0 + dR) - (2*R_star*acos(-y/R_star)*L_star);

        // Calculate Rayleigh number
        Ral_dome[i] = (g*0.01658*abs(Tw_dome[i] - Tl)*((R_int + y)^3)*Prl*rhol^2)/(liq.eta^2);
        Rav_dome[i] = (g*0.01658*abs(Tw_dome[i] - Tv)*((R_int - y)^3)*Prv*rhov^2)/(vap.eta^2);

        // Calculate Nusselt number
        Nul_dome[i] = 0.0605*(Ral_dome[i]^(1/3));
        Nuv_dome[i] = 0.364*2*R_int/(R_int - y)*(Rav_dome[i]^(1/4));

        // Calculate internal convective heat transfer coefficient
        h_int_l_dome[i] = (Nul_dome[i]*liq.lambda)/(R_int+y);
        h_int_v_dome[i] = (Nuv_dome[i]*vap.lambda)/(R_int-y);

        // Calculate equivalent radiative heat transfer coefficient
        h_rad_l_dome[i] = Modelica.Constants.sigma*0.09*(Tw_dome[i]^2 + Tl^2)*(Tw_dome[i] + Tl);
        h_rad_v_dome[i] = 0;  // Assumption

        // Calculate total heat transfer coefficient
        h_tot_l_dome[i] = h_int_l_dome[i] + h_rad_l_dome[i];
        h_tot_v_dome[i] = h_int_v_dome[i] + h_rad_v_dome[i];

        // Heat flows of the wall segments
        Qwl_dome[i] = h_tot_l_dome[i]*Awl_dome[i]*(Tw_dome[i] - Tl);
        Qwv_dome[i] = h_tot_v_dome[i]*Awv_dome[i]*(Tw_dome[i] - Tv);

        // Boundary conditions
        dome.ports[i,1].T = Tw_dome[i];
        Qwv_dome[i] + Qwl_dome[i] = dome.ports[i,1].Q_flow;
      end for;

      // Split the internal heat flow to the liquid and vapor, according to the volume
      A_frac = 1;//(Awl0+Awl1+Awl2+Awl3+Awl4+Awl5+Awl6+Awl7)/(Awt[1]+Awt[2]+Awt[3]+Awt[4]+Awt[5]+Awt[6]+Awt[7]+Awt[8]) "Liquid area fraction, neglecting the dome volumes";
      Qv_int = Q_int*(1 - A_frac);
      Ql_int = Q_int*A_frac;

      Qwl_tot = sum(Qwl) + sum(Qwl_dome);
      Qwv_tot = sum(Qwv) + sum(Qwv_dome);
      Ql_tot = Qwl_tot + Ql_int;
      Qv_tot = Qwv_tot + Qv_int;

      // Boundary conditions
      LH2_outlet.P = P;
      LH2_outlet.m_flow = ql;
      LH2_outlet.h_outflow = hl;
      VH2_outlet.P = P;
      VH2_outlet.m_flow = qv;
      VH2_outlet.h_outflow = hv;
      heat_port.T = Tv;
      Q_int = heat_port.Q_flow;

      // Sanity check
      assert(Ml > 1e-6, "Liquid fuel depleted");
      assert(Mv > 1e-6, "Gaseous fuel depleted");
      assert(Vl > 1e-6, "Liquid fuel depleted");
      assert(Vv > 1e-6, "Gaseous fuel depleted");
      assert(y < R_int - 1e-6, "Liquid level above max tank radius");
      assert(y > -R_int + 1e-6, "Liquid level below min tank radius");

    initial equation
      if initOpt == DynTherM.Choices.InitOpt.noInit then
        // do nothing

      elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
        if not noInitialPressure then
          P = P_start;
        end if;

        hl = hl_start;
        hv = hv_start;
        Vl = Vl_start;

      elseif initOpt == DynTherM.Choices.InitOpt.steadyState then
        if not noInitialPressure then
          der(P) = 0;
        end if;

        der(hl) = 0;
        der(hv) = 0;
        der(Vl) = 0;

      else
        assert(false, "Unsupported initialization option");
      end if;
      annotation (Dialog(tab="Initialisation"),
                        Dialog(tab="Initialisation"),
            Documentation(info="<HTML>
<p>Simplified model of a drum for drum boilers and fire-tube boilers. This model assumes
<ul>
<li>Thermodynamic equiibrium between the liquid, vapor, and metal wall
<li>Perfect separation of the liquid and vapor phase
</ul></p>
<p>The model has two state variables the pressure <code>p</code> and the liquid volume <code>Vl</code>. It is possible to extend it,
adding a specific geometry and the computation of the level from the liquid volume. In that case, one may want to use the level as a state.
</p>
</HTML>",       revisions="<html>
<ul>
<li><i>19 Feb 2019</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Adapted from old <code>Drum2States</code> model.</li>
</ul>
</html>"),Icon(graphics={Rectangle(
                extent={{-152,100},{122,-100}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Bitmap(extent={{-220,-136},{224,152}}, fileName="modelica://DynTherM/Figures/LH2Tank.png")}));
    end TankInternal;

    model CylinderHeatTransfer
      "Model of heat transfer from external environment through the fuselage to inner section of the tank"
      outer DynTherM.Components.Environment environment "Environmental properties";

      replaceable model HTC_ext =
        DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
        constrainedby
        DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
        "External convection correlation" annotation (choicesAllMatching=true);

      parameter Real coeff "Fraction of cylinder with active heat transfer";
      parameter Length L_tank "Length of the tank cylindrical section";
      parameter Length R_ext "External radius of the tank";
      parameter Length t_tank "Overall fuselage/tank thickness";
      parameter Length t_insulation "Overall insulation thickness";
      parameter Angle csi "Tilt angle of the surface wrt horizontal";
      parameter Irradiance E_tb "Beam component of the clear-sky solar irradiance";
      parameter Irradiance E_td "Diffuse component of the clear-sky solar irradiance";
      parameter Irradiance E_tr "Ground reflected component of the clear-sky solar irradiance";
      parameter Angle theta "Incidence angle";
      final parameter Mass m=fuselage.m + insulation.m "Mass";

      parameter Temperature T_start_tank
        "Fuselage/tank temperature start value" annotation (Dialog(tab="Initialization"));
      parameter Temperature T_start_insulation
        "Insulation temperature start value" annotation (Dialog(tab="Initialization"));

      final parameter Area A_ext=coeff*2*environment.pi*
          L_tank*R_ext "External tank area";

      Components.HeatTransfer.ExternalConvection extConvection(A=A_ext,
        redeclare model HTC = HTC_ext)
        annotation (Placement(transformation(extent={{16,66},{64,18}})));
      Components.HeatTransfer.WallRadiation wallRadiation(
        A=A_ext,
        csi=csi)
        annotation (Placement(transformation(extent={{-64,66},{-16,18}})));
      Components.HeatTransfer.SolarRadiation solarRadiation(
        E_tb_fixed=E_tb,
        E_td_fixed=E_td,
        E_tr_fixed=E_tr,
        theta_fixed=theta,
        csi=csi)
        annotation (Placement(transformation(extent={{-62,96},{-18,52}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatToInner annotation (
          Placement(transformation(extent={{-10,-110},{10,-90}}),
            iconTransformation(extent={{-10,-70},{10,-50}})));

      Components.HeatTransfer.TubeConduction fuselage(
        redeclare model Mat = DynTherM.Materials.Aluminium7075T6,
        coeff=coeff,
        L=L_tank,
        R_ext=R_ext,
        R_int=R_ext - t_tank,
        Tstart=T_start_tank,
        initOpt=environment.initOpt)
        annotation (Placement(transformation(extent={{-30,30},{30,-30}})));
      Components.HeatTransfer.TubeConduction insulation(
        redeclare model Mat = DynTherM.Materials.NFRSB31,
        coeff=coeff,
        L=L_tank,
        R_ext=R_ext - t_tank,
        R_int=R_ext - t_tank - t_insulation,
        Tstart=T_start_insulation,
        initOpt=environment.initOpt)
        annotation (Placement(transformation(extent={{-30,-20},{30,-80}})));
    equation
      connect(solarRadiation.inlet, wallRadiation.outlet)
        annotation (Line(points={{-40,60.36},{-40,45.36}}, color={191,0,0}));
      connect(fuselage.inlet, insulation.outlet)
        annotation (Line(points={{0,-10.2},{0,-39.8}}, color={191,0,0}));
      connect(insulation.inlet, heatToInner)
        annotation (Line(points={{0,-60.2},{0,-100}}, color={191,0,0}));
      connect(wallRadiation.inlet, fuselage.outlet) annotation (Line(points={{-40,33.84},
              {-40,10.2},{0,10.2}}, color={191,0,0}));
      connect(fuselage.outlet, extConvection.inlet) annotation (Line(points={{4.44089e-16,
              10.2},{40,10.2},{40,33.84}}, color={191,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(
              extent={{-100,-180},{100,20}},
              lineColor={0,0,0},
              startAngle=0,
              endAngle=180,
              closure=EllipseClosure.None,
              lineThickness=0.5),
            Ellipse(
              extent={{-80,-160},{80,0}},
              lineColor={0,0,0},
              startAngle=0,
              endAngle=180,
              closure=EllipseClosure.None,
              lineThickness=0.5),
            Line(
              points={{-100,-80},{-80,-80}},
              color={0,0,0},
              thickness=0.5),
            Line(
              points={{80,-80},{100,-80}},
              color={0,0,0},
              thickness=0.5),
            Ellipse(
              extent={{-70,-150},{68,-12}},
              lineColor={28,108,200},
              startAngle=0,
              endAngle=80,
              closure=EllipseClosure.None,
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{70,-150},{-68,-12}},
              lineColor={28,108,200},
              startAngle=0,
              endAngle=80,
              closure=EllipseClosure.None,
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-10,-12},{-16,-18}}, color={28,108,200}),
            Line(points={{-10,-12},{-18,-10}}, color={28,108,200}),
            Line(points={{10,-12},{16,-18}}, color={28,108,200}),
            Line(points={{10,-12},{18,-10}}, color={28,108,200}),
            Line(points={{-20,24},{-20,80}},  color={238,46,47}),
            Line(points={{-14,34},{-20,24}},  color={238,46,47}),
            Line(points={{-26,34},{-20,24}},  color={238,46,47}),
            Line(points={{-40,18},{-40,74}},  color={238,46,47}),
            Line(points={{-34,28},{-40,18}},  color={238,46,47}),
            Line(points={{-46,28},{-40,18}},  color={238,46,47}),
            Line(points={{20,24},{20,80}},    color={238,46,47}),
            Line(points={{26,34},{20,24}},    color={238,46,47}),
            Line(points={{14,34},{20,24}},    color={238,46,47}),
            Line(points={{40,18},{40,74}},    color={238,46,47}),
            Line(points={{46,28},{40,18}},    color={238,46,47}),
            Line(points={{34,28},{40,18}},    color={238,46,47}),
            Text(
              extent={{-30,-16},{28,-46}},
              lineColor={0,0,0},
              lineThickness=0.5,
              textString="FUSELAGE")}),                              Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end CylinderHeatTransfer;

    model DomeHeatTransfer
      "Model of heat transfer from external environment through the dome to inner section of the tank"
      outer DynTherM.Components.Environment environment "Environmental properties";

      replaceable model HTC_ext =
        DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
        constrainedby
        DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
        "External convection correlation" annotation (choicesAllMatching=true);

      parameter Real coeff "Fraction of cylinder with active heat transfer";
      parameter Length L_tank "Length of the tank cylindrical section";
      parameter Length R_ext "External radius of the tank";
      parameter Length t_tank "Overall fuselage/tank thickness";
      parameter Length t_insulation "Overall insulation thickness";
      parameter Real AR "Aspect Ratio of the sphere/dome";
      parameter Angle csi "Tilt angle of the surface wrt horizontal";
      parameter Irradiance E_tb "Beam component of the clear-sky solar irradiance";
      parameter Irradiance E_td "Diffuse component of the clear-sky solar irradiance";
      parameter Irradiance E_tr "Ground reflected component of the clear-sky solar irradiance";
      parameter Angle theta "Incidence angle";
      final parameter Mass m=fuselage.m + insulation.m "Mass";

      parameter Temperature T_start_tank
        "Fuselage/tank temperature start value" annotation (Dialog(tab="Initialization"));
      parameter Temperature T_start_insulation
        "Insulation temperature start value" annotation (Dialog(tab="Initialization"));

      final parameter Area A_ext=coeff*2*environment.pi*
          L_tank*R_ext "External tank area";

      Components.HeatTransfer.ExternalConvection extConvection(A=A_ext,
        redeclare model HTC = HTC_ext)
        annotation (Placement(transformation(extent={{16,66},{64,18}})));
      Components.HeatTransfer.WallRadiation wallRadiation(
        A=A_ext,
        csi=csi)
        annotation (Placement(transformation(extent={{-64,66},{-16,18}})));
      Components.HeatTransfer.SolarRadiation solarRadiation(
        E_tb_fixed=E_tb,
        E_td_fixed=E_td,
        E_tr_fixed=E_tr,
        theta_fixed=theta,
        csi=csi)
        annotation (Placement(transformation(extent={{-62,96},{-18,52}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatToInner annotation (
          Placement(transformation(extent={{-10,-110},{10,-90}}),
            iconTransformation(extent={{-10,-70},{10,-50}})));

      Components.HeatTransfer.SphereConduction fuselage(
        redeclare model Mat = DynTherM.Materials.Aluminium7075T6,
        coeff=coeff,
        R_ext=R_ext,
        R_int=R_ext - t_tank,
        AR=AR,
        Tstart=T_start_tank,
        initOpt=environment.initOpt)
        annotation (Placement(transformation(extent={{-30,30},{30,-30}})));
      Components.HeatTransfer.SphereConduction insulation(
        redeclare model Mat = DynTherM.Materials.NFRSB31,
        coeff=coeff,
        AR=AR,
        R_ext=R_ext - t_tank,
        R_int=R_ext - t_tank - t_insulation,
        Tstart=T_start_insulation,
        initOpt=environment.initOpt)
        annotation (Placement(transformation(extent={{-30,-20},{30,-80}})));
    equation
      connect(solarRadiation.inlet, wallRadiation.outlet)
        annotation (Line(points={{-40,60.36},{-40,45.36}}, color={191,0,0}));
      connect(fuselage.inlet, insulation.outlet)
        annotation (Line(points={{0,-10.2},{0,-39.8}}, color={191,0,0}));
      connect(insulation.inlet, heatToInner)
        annotation (Line(points={{0,-60.2},{0,-100}}, color={191,0,0}));
      connect(wallRadiation.inlet, fuselage.outlet) annotation (Line(points={{-40,33.84},
              {-40,10.2},{0,10.2}}, color={191,0,0}));
      connect(fuselage.outlet, extConvection.inlet) annotation (Line(points={{4.44089e-16,
              10.2},{40,10.2},{40,33.84}}, color={191,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(
              extent={{-100,-180},{100,20}},
              lineColor={0,0,0},
              startAngle=0,
              endAngle=180,
              closure=EllipseClosure.None,
              lineThickness=0.5),
            Ellipse(
              extent={{-80,-160},{80,0}},
              lineColor={0,0,0},
              startAngle=0,
              endAngle=180,
              closure=EllipseClosure.None,
              lineThickness=0.5),
            Line(
              points={{-100,-80},{-80,-80}},
              color={0,0,0},
              thickness=0.5),
            Line(
              points={{80,-80},{100,-80}},
              color={0,0,0},
              thickness=0.5),
            Ellipse(
              extent={{-70,-150},{68,-12}},
              lineColor={28,108,200},
              startAngle=0,
              endAngle=80,
              closure=EllipseClosure.None,
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{70,-150},{-68,-12}},
              lineColor={28,108,200},
              startAngle=0,
              endAngle=80,
              closure=EllipseClosure.None,
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-10,-12},{-16,-18}}, color={28,108,200}),
            Line(points={{-10,-12},{-18,-10}}, color={28,108,200}),
            Line(points={{10,-12},{16,-18}}, color={28,108,200}),
            Line(points={{10,-12},{18,-10}}, color={28,108,200}),
            Line(points={{-20,24},{-20,80}},  color={238,46,47}),
            Line(points={{-14,34},{-20,24}},  color={238,46,47}),
            Line(points={{-26,34},{-20,24}},  color={238,46,47}),
            Line(points={{-40,18},{-40,74}},  color={238,46,47}),
            Line(points={{-34,28},{-40,18}},  color={238,46,47}),
            Line(points={{-46,28},{-40,18}},  color={238,46,47}),
            Line(points={{20,24},{20,80}},    color={238,46,47}),
            Line(points={{26,34},{20,24}},    color={238,46,47}),
            Line(points={{14,34},{20,24}},    color={238,46,47}),
            Line(points={{40,18},{40,74}},    color={238,46,47}),
            Line(points={{46,28},{40,18}},    color={238,46,47}),
            Line(points={{34,28},{40,18}},    color={238,46,47}),
            Text(
              extent={{-30,-16},{28,-46}},
              lineColor={0,0,0},
              lineThickness=0.5,
              textString="FUSELAGE")}),                              Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end DomeHeatTransfer;

    model TankPressureController
      parameter Pressure P_vent "Venting pressure";
      parameter MassFlowRate f_vent_max=100 "Maximum allowable venting mass flow rate";

      Modelica.Blocks.Sources.RealExpression Vent_Pressure(y=P_vent)
        annotation (Placement(transformation(extent={{-98,-20},{-70,0}})));
      Modelica.Blocks.Sources.RealExpression Zero
        annotation (Placement(transformation(extent={{-76,50},{-54,68}})));
      Modelica.Blocks.Math.Max max1
        annotation (Placement(transformation(extent={{-28,2},{-12,18}})));
      Modelica.Blocks.Math.Feedback Subtract2 annotation (Placement(visible=true,
            transformation(
            origin={-51,-10},
            extent={{-7,8},{7,-8}},
            rotation=90)));
      Modelica.Blocks.Math.Gain gain(k=-0.00012)
                                             annotation (Placement(transformation(
            extent={{8,-8},{-8,8}},
            rotation=180,
            origin={8,10})));
      Modelica.Blocks.Math.Min min1 annotation (Placement(transformation(
            extent={{-8,-8},{8,8}},
            rotation=0,
            origin={48,54})));
      Modelica.Blocks.Math.Add add annotation (Placement(transformation(
            extent={{-8,-8},{8,8}},
            rotation=90,
            origin={30,30})));
      Modelica.Blocks.Interfaces.RealInput Tank_Pressure annotation (Placement(
            transformation(extent={{-120,-80},{-80,-40}}),  iconTransformation(
              extent={{-120,-80},{-80,-40}})));


      Modelica.Blocks.Math.Max max2
        annotation (Placement(transformation(extent={{70,50},{90,70}})));
      Modelica.Blocks.Sources.RealExpression Max_f_vent(y=f_vent_max)
        annotation (Placement(transformation(extent={{-80,70},{-52,90}})));
      Modelica.Blocks.Math.Gain gain1(k=-1)  annotation (Placement(transformation(
            extent={{8,-8},{-8,8}},
            rotation=180,
            origin={-32,80})));

      Modelica.Blocks.Interfaces.RealInput External_Massflow annotation (
          Placement(transformation(extent={{-120,-50},{-80,-10}}),
            iconTransformation(extent={{-120,40},{-80,80}})));
      Modelica.Blocks.Interfaces.RealOutput Required_Massflow annotation (
          Placement(transformation(extent={{96,40},{136,80}}),
            iconTransformation(extent={{80,-20},{120,20}})));
    equation
      connect(Subtract2.y, max1.u2) annotation (Line(points={{-51,-3.7},{-51,5.2},{-29.6,
              5.2}}, color={0,0,127}));
      connect(max1.y, gain.u)
        annotation (Line(points={{-11.2,10},{-1.6,10}},color={0,0,127}));
      connect(Vent_Pressure.y, Subtract2.u2)
        annotation (Line(points={{-68.6,-10},{-57.4,-10}},
                                                         color={0,0,127}));
      connect(Zero.y, max1.u1) annotation (Line(points={{-52.9,59},{-44,59},{-44,14.8},
              {-29.6,14.8}},color={0,0,127}));
      connect(Zero.y, min1.u1) annotation (Line(points={{-52.9,59},{38,59},{38,58},{
              38.4,58},{38.4,58.8}},
                            color={0,0,127}));
      connect(add.y, min1.u2) annotation (Line(points={{30,38.8},{30,49.2},{38.4,49.2}},
            color={0,0,127}));
      connect(gain.y, add.u1) annotation (Line(points={{16.8,10},{25.2,10},{25.2,20.4}},
            color={0,0,127}));
      connect(Tank_Pressure, Subtract2.u1) annotation (Line(points={{-100,-60},
              {-51,-60},{-51,-15.6}},
                            color={0,0,127}));
      connect(min1.y, max2.u2) annotation (Line(points={{56.8,54},{68,54}},
                            color={0,0,127}));
      connect(Max_f_vent.y, gain1.u)
        annotation (Line(points={{-50.6,80},{-41.6,80}},
                                                      color={0,0,127}));
      connect(gain1.y, max2.u1) annotation (Line(points={{-23.2,80},{60,80},{60,66},
              {68,66}},           color={0,0,127}));
      connect(External_Massflow, add.u2) annotation (Line(points={{-100,-30},{
              34.8,-30},{34.8,20.4}}, color={0,0,127}));
      connect(max2.y, Required_Massflow)
        annotation (Line(points={{91,60},{116,60}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-60,54},{60,-40}},
              lineColor={0,0,0},
              textString="Tank
Pressure
Control")}),        Diagram(coordinateSystem(preserveAspectRatio=false)));
    end TankPressureController;
  end Subsystems;
end HydrogenTank;
