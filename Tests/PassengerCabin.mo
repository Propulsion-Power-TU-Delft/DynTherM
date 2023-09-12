within ThermalManagement.Tests;
model PassengerCabin

  parameter Real N_pax=0 "Number of passengers inside the aircraft" annotation (Dialog(tab="Flight conditions"));
  parameter Real N_crew=6 "Number of cabin crew members inside the aircraft" annotation (Dialog(tab="Flight conditions"));
  parameter Real N_pilots=3 "Number of pilots inside the aircraft" annotation (Dialog(tab="Flight conditions"));
  parameter Modelica.Units.SI.HeatFlowRate Q_el=1200
    "Internal heat load due to flight deck electronics" annotation (Dialog(tab="Flight conditions"));
  parameter Modelica.Units.SI.HeatFlowRate Q_galley=0
    "Internal heat load due to galleys" annotation (Dialog(tab="Flight conditions"));
  parameter Modelica.Units.SI.HeatFlowRate Q_avionics=0
    "Internal heat load due to avionics"  annotation (Dialog(tab="Flight conditions"));
  parameter Real cabinLights=100 "Percentage of usage [0-100] of cabin lights" annotation (Dialog(tab="Flight conditions"));
  parameter Real inFlightEntertainment=0 "Percentage of usage [0-100] of in-flight entertainment" annotation (Dialog(tab="Flight conditions"));
  parameter Real rec_target=0.2
    "Target ratio among the recirculated airflow and the total airflow";
  parameter Modelica.Units.SI.Length R_fuselage=2.07
    "External radius of the fuselage";
  parameter Modelica.Units.SI.Length R_cockpit=R_fuselage "External radius of the cockpit";
  parameter Modelica.Units.SI.Length L_fuselage=26.9 "Length of the fuselage";
  parameter Modelica.Units.SI.Length L_cockpit=3.5 "Length of the cockpit";
  parameter Modelica.Units.SI.Length L_cargo=4.95 + 9.8 "Length of cargo bay: fwd + aft";
  parameter Modelica.Units.SI.Length L_EEbay=4 "Length of electronics bay";
  parameter Modelica.Units.SI.Volume V_cabin=139 "Passenger cabin internal volume";
  parameter Modelica.Units.SI.Volume V_cargo=15.56 + 20.77
    "Cargo bay internal volume: fwd + aft";
  parameter Modelica.Units.SI.Volume V_cockpit=9 "Cockpit internal volume";
  parameter Modelica.Units.SI.Volume V_EEbay=V_cockpit "Electronics bay internal volume";
  parameter Modelica.Units.SI.Length L_window=0.23 "Cabin window length";
  parameter Modelica.Units.SI.Length H_window=0.33 "Cabin window height";
  parameter Modelica.Units.SI.Area A_windshield_front=0.52*2
    "Surface of windshield - frontal section";
  parameter Modelica.Units.SI.Length L_windshield_lat=(0.3+0.36)/0.5
    "Windshield length - lateral section";
  parameter Modelica.Units.SI.Length H_windshield_lat=0.5
    "Windshield height - lateral section";
  parameter Modelica.Units.SI.Length L_windshield_front=(0.3+0.36)/0.5
    "Windshield length - frontal section";
  parameter Modelica.Units.SI.Length H_windshield_front=0.5
    "Windshield height - frontal section";
  parameter Integer Nw_side=30 "Number of windows per fuselage side";
  parameter Modelica.Units.SI.Length H_fl=1.3 "Floor height";
  parameter Modelica.Units.SI.SpecificHeatCapacity c_cabin=1000
    "Specific heat capacity of cabin interior";
  parameter Modelica.Units.SI.SpecificHeatCapacity c_cockpit=c_cabin
    "Specific heat capacity of cockpit interior";
  parameter Modelica.Units.SI.Mass m_cabin=20*180 "Mass of cabin interior";
  parameter Modelica.Units.SI.Mass m_cockpit=20*4 "Mass of cockpit interior";
  parameter Modelica.Units.SI.Area A_cabin=2*180
    "Heat transfer surface of cabin interior";
  parameter Modelica.Units.SI.Area A_cockpit=2*4
    "Heat transfer surface of cockpit interior";
  parameter Modelica.Units.SI.Length t_upper=0.1
    "Overall fuselage thickness - upper part";
  parameter Modelica.Units.SI.Length t_lower=0.1
    "Overall fuselage thickness - lower part";

  parameter Modelica.Units.SI.Irradiance E_tb_1=632.22
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_1=0
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_1=0
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_1=1.3004797322460149
    "Fixed value of incidence angle - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tb_2=965.24
    "Fixed value of solar irradiance - section 2" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_2=0
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_2=0
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_2=0.606973153966068
    "Fixed value of incidence angle - section 2" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tb_3=951.14
    "Fixed value of solar irradiance - section 3" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_3=0
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_3=0
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_3=0.46335000981945457
    "Fixed value of incidence angle - section 3" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tb_4=597.64
    "Fixed value of solar irradiance - section 4" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_4=0
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_4=0
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_4=1.1110416952345503
    "Fixed value of incidence angle - section 4" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tb_5=371.98
    "Fixed value of solar irradiance - section 5" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_td_5=0
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Irradiance E_tr_5=0
    "Fixed value of solar irradiance - section 1" annotation (Dialog(tab="Solar radiation"));
  parameter Modelica.Units.SI.Angle theta_5=1.8411129213437782
    "Fixed value of incidence angle - section 5" annotation (Dialog(tab="Solar radiation"));

  parameter Modelica.Units.SI.Length L_pipe_cab=L_fuselage/2
    "Length of air distribution pipe - cabin" annotation (Dialog(tab="Air distribution"));
  parameter Modelica.Units.SI.Length D_pipe_cab=0.18
    "Diameter of air distribution pipe - cabin" annotation (Dialog(tab="Air distribution"));
  parameter Modelica.Units.SI.Length L_pipe_fd=L_fuselage/2
    "Length of air distribution pipe - cockpit" annotation (Dialog(tab="Air distribution"));
  parameter Modelica.Units.SI.Length D_pipe_fd=0.08
    "Diameter of air distribution pipe - cockpit" annotation (Dialog(tab="Air distribution"));
  parameter Modelica.Units.SI.Volume V_mixingManifold=0.5
    "Mixing manifold internal volume" annotation (Dialog(tab="Air distribution"));
  parameter Real eta_is=0.65 "Fan: isentropic efficiency at design point" annotation (Dialog(tab="Air distribution"));
  parameter Real eta_m=0.95 "Fan: mechanical efficiency" annotation (Dialog(tab="Air distribution"));
  parameter Modelica.Units.SI.AngularVelocity omega_nom=157.08
    "Fan: nominal rotational speed" annotation (Dialog(tab="Air distribution"));
  parameter Modelica.Units.SI.VolumeFlowRate volFlow_nom=0.7
    "Fan: nominal volumetric flow rate" annotation (Dialog(tab="Air distribution"));
  parameter Modelica.Units.SI.SpecificEnergy Head_nom=400 "Fan: nominal head" annotation (Dialog(tab="Air distribution"));
  parameter ThermalManagement.CustomUnits.HydraulicConductance Kv=0.005
    "Outflow valve: hydraulic conductance" annotation (Dialog(tab="Air distribution"));
  parameter ThermalManagement.CustomUnits.HydraulicResistance R_HEPA=2000
    "Filter: hydraulic Resistance" annotation (Dialog(tab="Air distribution"));
  parameter ThermalManagement.CustomUnits.HydraulicResistance R_dado=100
    "Dado panel: hydraulic Resistance" annotation (Dialog(tab="Air distribution"));

  parameter Modelica.Units.SI.Temperature Tstart_fuselage=323.15
    "Fuselage temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature Tstart_floor=313.15
    "Floor temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature Tstart_wall=313.15
    "Cabin wall temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature Tstart_cabin=313.15
    "Cabin temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature Tstart_cargo=313.15
    "Cargo bay temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature Tstart_flightDeck=313.15
    "Flight deck temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature Tstart_mixingManifold=313.15
    "Mixing manifold temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure Pstart_cabin=101325
    "Cabin pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure Pstart_cargo=101325
    "Cargo bay pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure Pstart_flightDeck=101325
    "Flight deck pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure Pstart_mixingManifold=101325
    "Mixing manifold pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Density rho_start_fan=1.1 "Density - start value"
    annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.SpecificEnthalpy h_start_fan=1e5
    "Specific enthalpy - start value" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure_cabin=false "Remove initial equation on pressure - cabin" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature_cabin=false "Remove initial equation on temperature - cabin" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialPressure_cargo=false "Remove initial equation on pressure - cargo" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature_cargo=false "Remove initial equation on temperature - cargo" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialPressure_cockpit=false "Remove initial equation on pressure - cockpit" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature_cockpit=false "Remove initial equation on temperature - cockpit" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  final parameter Modelica.Units.SI.HeatFlux Q_light_m2=11.2
    "Internal heat load due to cabin lights / cabin floor m2";
  final parameter Modelica.Units.SI.HeatFlowRate Q_ife_pax=30
    "Internal heat load due to in-flight entertainment / pax";
  final parameter Modelica.Units.SI.Area A_floor=cabin.W_fl*L_fuselage
    "Approximated surface area of cabin floor";
  final parameter Modelica.Units.SI.Area A_floor_cargo=cabin.W_fl*L_cargo
    "Approximated surface area of cargo floor";
  final parameter Modelica.Units.SI.Area A_wall=theta/2*R_fuselage^2 + cabin.W_fl
      *(R_fuselage - H_fl)/2 "Approximated surface area of cabin wall";
  final parameter Modelica.Units.SI.Angle theta=atan((R_fuselage - H_fl)/
      R_fuselage);
  final parameter Modelica.Units.SI.HeatFlowRate Q_utilities=
      inFlightEntertainment/100*N_pax*Q_ife_pax + cabinLights/100*
      Q_light_m2*A_floor + Q_galley;

  inner Components.Environment environment(
    Mach_inf=0,
    Altitude=0,
    ISA_plus=23,
    phi_amb=0.22,
    phi_amb_ground=0.22,
    T_ground(displayUnit="degC") = 323.15,
    use_ext_sw=true,
    allowFlowReversal=false,
    initOpt=ThermalManagement.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{66,66},{100,100}})));
  Components.MassTransfer.SourceMassFlow                   cabinTrimFlow(
    use_in_massFlow=false,
    use_in_T=false,
    use_in_Xw=false) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-24,80})));
  Components.MassTransfer.PressureSink pressureSink
    annotation (Placement(transformation(extent={{20,-90},{40,-70}})));
  Systems.Aircraft.Subsystems.PassengerCabin cabin(
    redeclare model HTC_int =
        ThermalManagement.Components.HeatTransfer.HTCorrelations.InternalConvection.Cylinder,
    redeclare model HTC_ext =
        ThermalManagement.Components.HeatTransfer.HTCorrelations.ExternalConvection.AircraftOnGroundFree
        (R_ext=R_fuselage),
    N_occupants={N_pax,N_crew,0},
    Q_int=Q_utilities,
    L_fuselage=L_fuselage,
    R_ext=R_fuselage,
    V_cabin=V_cabin,
    c_cabin=c_cabin,
    m_cabin=m_cabin,
    A_cabin=A_cabin,
    A_floor=A_floor,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    t_cabin=t_upper,
    E_tb_1=E_tb_1,
    E_td_1=E_td_1,
    E_tr_1=E_tr_1,
    theta_1=theta_1,
    E_tb_2=E_tb_2,
    E_td_2=E_td_2,
    E_tr_2=E_tr_2,
    theta_2=theta_2,
    E_tb_3=E_tb_3,
    E_td_3=E_td_3,
    E_tr_3=E_tr_3,
    theta_3=theta_3,
    E_tb_4=E_tb_4,
    E_td_4=E_td_4,
    E_tr_4=E_tr_4,
    theta_4=theta_4,
    E_tb_5=E_tb_5,
    E_td_5=E_td_5,
    E_tr_5=E_tr_5,
    theta_5=theta_5,
    Tstart_fuselage=Tstart_fuselage,
    Tstart_cabin=Tstart_cabin,
    Pstart_cabin=Pstart_cabin,
    noInitialPressure=noInitialPressure_cabin,
    noInitialTemperature=noInitialTemperature_cabin)
    annotation (Placement(transformation(extent={{-22,-20},{22,24}})));

equation
  connect(cabinTrimFlow.outlet, cabin.cabinInflow)
    annotation (Line(points={{-14,80},{0,80},{0,24}}, color={0,0,0}));
  connect(cabin.cabinToCargo, pressureSink.inlet)
    annotation (Line(points={{0,-20},{0,-80},{20,-80}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PassengerCabin;
