within DynTherM.Systems.HydrogenTank;
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
