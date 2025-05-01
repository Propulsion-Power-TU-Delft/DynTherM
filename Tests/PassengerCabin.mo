within DynTherM.Tests;
model PassengerCabin

  parameter Real N_pax=0 "Number of passengers inside the aircraft" annotation (Dialog(tab="Flight conditions"));
  parameter Real N_crew=6 "Number of cabin crew members inside the aircraft" annotation (Dialog(tab="Flight conditions"));
  parameter Real N_pilots=3 "Number of pilots inside the aircraft" annotation (Dialog(tab="Flight conditions"));
  parameter HeatFlowRate Q_el=1200
    "Internal heat load due to flight deck electronics" annotation (Dialog(tab="Flight conditions"));
  parameter HeatFlowRate Q_galley=0
    "Internal heat load due to galleys" annotation (Dialog(tab="Flight conditions"));
  parameter HeatFlowRate Q_avionics=0
    "Internal heat load due to avionics"  annotation (Dialog(tab="Flight conditions"));
  parameter Real cabinLights=100 "Percentage of usage [0-100] of cabin lights" annotation (Dialog(tab="Flight conditions"));
  parameter Real inFlightEntertainment=0 "Percentage of usage [0-100] of in-flight entertainment" annotation (Dialog(tab="Flight conditions"));
  parameter Real rec_target=0.2
    "Target ratio among the recirculated airflow and the total airflow";
  parameter Length R_fuselage=2.07
    "External radius of the fuselage";
  parameter Length R_cockpit=R_fuselage "External radius of the cockpit";
  parameter Length L_fuselage=26.9 "Length of the fuselage";
  parameter Length L_cockpit=3.5 "Length of the cockpit";
  parameter Length L_cargo=4.95 + 9.8 "Length of cargo bay: fwd + aft";
  parameter Length L_EEbay=4 "Length of electronics bay";
  parameter Volume V_cabin=139 "Passenger cabin internal volume";
  parameter Volume V_cargo=15.56 + 20.77
    "Cargo bay internal volume: fwd + aft";
  parameter Volume V_cockpit=9 "Cockpit internal volume";
  parameter Volume V_EEbay=V_cockpit "Electronics bay internal volume";
  parameter Length L_window=0.23 "Cabin window length";
  parameter Length H_window=0.33 "Cabin window height";
  parameter Area A_windshield_front=0.52*2
    "Surface of windshield - frontal section";
  parameter Length L_windshield_lat=(0.3+0.36)/0.5
    "Windshield length - lateral section";
  parameter Length H_windshield_lat=0.5
    "Windshield height - lateral section";
  parameter Length L_windshield_front=(0.3+0.36)/0.5
    "Windshield length - frontal section";
  parameter Length H_windshield_front=0.5
    "Windshield height - frontal section";
  parameter Integer Nw_side=30 "Number of windows per fuselage side";
  parameter Length H_fl=1.3 "Floor height";
  parameter SpecificHeatCapacity c_cabin=1000
    "Specific heat capacity of cabin interior";
  parameter SpecificHeatCapacity c_cockpit=c_cabin
    "Specific heat capacity of cockpit interior";
  parameter Mass m_cabin=20*180 "Mass of cabin interior";
  parameter Mass m_cockpit=20*4 "Mass of cockpit interior";
  parameter Area A_cabin=2*180
    "Heat transfer surface of cabin interior";
  parameter Area A_cockpit=2*4
    "Heat transfer surface of cockpit interior";
  parameter Length t_upper=0.1
    "Overall fuselage thickness - upper part";
  parameter Length t_lower=0.1
    "Overall fuselage thickness - lower part";

  parameter Irradiance E_tb[5]={632.22,965.24,951.14,597.64,371.98}
    "Fixed value of solar irradiance" annotation (Dialog(tab="Solar radiation"));
  parameter Irradiance E_td[5]={0,0,0,0,0}
    "Fixed value of solar irradiance" annotation (Dialog(tab="Solar radiation"));
  parameter Irradiance E_tr[5]={0,0,0,0,0}
    "Fixed value of solar irradiance" annotation (Dialog(tab="Solar radiation"));
  parameter Angle theta[5]={1.3004797322460149,0.606973153966068,0.46335000981945457,1.1110416952345503,1.8411129213437782}
    "Fixed value of incidence angle" annotation (Dialog(tab="Solar radiation"));

  parameter Length L_pipe_cab=L_fuselage/2
    "Length of air distribution pipe - cabin" annotation (Dialog(tab="Air distribution"));
  parameter Length D_pipe_cab=0.18
    "Diameter of air distribution pipe - cabin" annotation (Dialog(tab="Air distribution"));
  parameter Length L_pipe_fd=L_fuselage/2
    "Length of air distribution pipe - cockpit" annotation (Dialog(tab="Air distribution"));
  parameter Length D_pipe_fd=0.08
    "Diameter of air distribution pipe - cockpit" annotation (Dialog(tab="Air distribution"));
  parameter Volume V_mixingManifold=0.5
    "Mixing manifold internal volume" annotation (Dialog(tab="Air distribution"));
  parameter Real eta_is=0.65 "Fan: isentropic efficiency at design point" annotation (Dialog(tab="Air distribution"));
  parameter Real eta_m=0.95 "Fan: mechanical efficiency" annotation (Dialog(tab="Air distribution"));
  parameter AngularVelocity omega_nom=157.08
    "Fan: nominal rotational speed" annotation (Dialog(tab="Air distribution"));
  parameter VolumeFlowRate volFlow_nom=0.7
    "Fan: nominal volumetric flow rate" annotation (Dialog(tab="Air distribution"));
  parameter SpecificEnergy Head_nom=400 "Fan: nominal head" annotation (Dialog(tab="Air distribution"));
  parameter DynTherM.CustomUnits.HydraulicConductance Kv=0.005
    "Outflow valve: hydraulic conductance"
    annotation (Dialog(tab="Air distribution"));
  parameter DynTherM.CustomUnits.HydraulicResistance R_HEPA=2000
    "Filter: hydraulic Resistance" annotation (Dialog(tab="Air distribution"));
  parameter DynTherM.CustomUnits.HydraulicResistance R_dado=100
    "Dado panel: hydraulic Resistance"
    annotation (Dialog(tab="Air distribution"));

  parameter Temperature Tstart_fuselage=323.15
    "Fuselage temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_floor=313.15
    "Floor temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_wall=313.15
    "Cabin wall temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_cabin=313.15
    "Cabin temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_cargo=313.15
    "Cargo bay temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_flightDeck=313.15
    "Flight deck temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart_mixingManifold=313.15
    "Mixing manifold temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure Pstart_cabin=101325
    "Cabin pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure Pstart_cargo=101325
    "Cargo bay pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure Pstart_flightDeck=101325
    "Flight deck pressure start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure Pstart_mixingManifold=101325
    "Mixing manifold pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Density rho_start_fan=1.1 "Density - start value"
    annotation (Dialog(tab="Initialization"));
  parameter SpecificEnthalpy h_start_fan=1e5
    "Specific enthalpy - start value" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure_cabin=false "Remove initial equation on pressure - cabin" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature_cabin=false "Remove initial equation on temperature - cabin" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialPressure_cargo=false "Remove initial equation on pressure - cargo" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature_cargo=false "Remove initial equation on temperature - cargo" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialPressure_cockpit=false "Remove initial equation on pressure - cockpit" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Boolean noInitialTemperature_cockpit=false "Remove initial equation on temperature - cockpit" annotation (Dialog(tab="Initialization"),choices(checkBox=true));

  final parameter HeatFlux Q_light_m2=11.2
    "Internal heat load due to cabin lights / cabin floor m2";
  final parameter HeatFlowRate Q_ife_pax=30
    "Internal heat load due to in-flight entertainment / pax";

  Area A_floor "Approximated surface area of cabin floor";
  Area A_floor_cargo "Approximated surface area of cargo floor";
  Area A_wall "Approximated surface area of cabin wall";
  Angle angle;
  HeatFlowRate Q_utilities;

  inner Components.Environment environment(
    V_inf_di=0,
    ISA_plus=23,
    phi_amb=0.22,
    phi_amb_ground=0.22,
    T_ground(displayUnit="degC") = 323.15,
    use_ext_sw=true,
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{66,66},{100,100}})));
  BoundaryConditions.ZeroDimensional.flow_source cabinTrimFlow(use_in_massFlow=
        false, use_in_T=false) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-24,80})));
  BoundaryConditions.ZeroDimensional.pressure_sink pressureSink
    annotation (Placement(transformation(extent={{20,-90},{40,-70}})));
  Systems.Aircraft.Subsystems.PassengerCabin cabin(
    redeclare model HTC_int =
        DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.Cylinder,
    redeclare model HTC_ext =
        DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.AircraftOnGroundFree
        (R_ext=R_fuselage),
    N_occupants={N_pax,N_crew,0},
    Q_int=Q_utilities,
    L_cabin=L_fuselage,
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
    E_tb=E_tb,
    E_td=E_td,
    E_tr=E_tr,
    theta=theta,
    Tstart_fuselage=Tstart_fuselage,
    Tstart_cabin=Tstart_cabin,
    Pstart_cabin=Pstart_cabin,
    noInitialPressure=true,
    noInitialTemperature=false)
    annotation (Placement(transformation(extent={{-22,-20},{22,24}})));

equation
  A_floor = cabin.W_fl*L_fuselage;
  A_floor_cargo = cabin.W_fl*L_cargo;
  A_wall = angle/2*R_fuselage^2 + cabin.W_fl*(R_fuselage - H_fl)/2;
  angle = atan((R_fuselage - H_fl)/R_fuselage);
  Q_utilities = inFlightEntertainment/100*N_pax*Q_ife_pax + cabinLights/100*
    Q_light_m2*A_floor + Q_galley;

  connect(cabinTrimFlow.outlet, cabin.cabinInflow)
    annotation (Line(points={{-14,80},{0,80},{0,24}}, color={0,0,0}));
  connect(cabin.cabinToCargo, pressureSink.inlet)
    annotation (Line(points={{0,-20},{0,-80},{20,-80}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PassengerCabin;
