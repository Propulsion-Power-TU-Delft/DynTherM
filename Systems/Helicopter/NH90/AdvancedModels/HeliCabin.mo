within ThermalManagement.Systems.Helicopter.NH90.AdvancedModels;
model HeliCabin
  "Helicopter cabin advanced model made to match with basic NH90 Airbus"
  outer Components.Environment environment;
  parameter Modelica.Units.SI.Length LX=4.85 "Fuselage X-length"
    annotation (Dialog(tab="Dimensions", group="General"));
  parameter Modelica.Units.SI.Length WY=2.6 "Fuselage Y-length"
    annotation (Dialog(tab="Dimensions", group="General"));
  parameter Modelica.Units.SI.Length HZ=2.8 "Fuselage Z-length"
    annotation (Dialog(tab="Dimensions", group="General"));

  parameter Modelica.Units.SI.Length t_fus=0.03 "Fuselage wall thickness"
    annotation (Dialog(tab="Dimensions", group="Fuselage"));
  parameter Modelica.Units.SI.Area A_floor=12.6 "Floor surface area";
  parameter Modelica.Units.SI.Length t_floor=0.025 "Floor thickness"
    annotation (Dialog(tab="Dimensions", group="Fuselage"));
  parameter Modelica.Units.SI.Area A_duct=2 "Duct surface area"
    annotation (Dialog(tab="Dimensions", group="Fuselage"));
  parameter Modelica.Units.SI.Length t_duct=0.005 "Duct thickness"
    annotation (Dialog(tab="Dimensions", group="Fuselage"));
  parameter Modelica.Units.SI.Angle csi_fus=90*pi/180
    "Angle of panels (acute)"
    annotation (Dialog(tab="Dimensions", group="Fuselage"));

  parameter Modelica.Units.SI.Area A_engine=4.2 "Engine area"
    annotation (Dialog(tab="Dimensions", group="Engine"));
  parameter Modelica.Units.SI.Area A_btp=4.2 "Transmission area"
    annotation (Dialog(tab="Dimensions", group="Engine"));

  parameter Modelica.Units.SI.Area A_window=2.5 "Window area per side"
    annotation (Dialog(tab="Dimensions", group="Window"));
  //parameter Modelica.SIunits.Area Ap_window=0.25 "Window projected area" annotation(Dialog(tab="Dimensions", group="Window"));
  parameter Modelica.Units.SI.Length t_window=0.01 "Window wall thickness"
    annotation (Dialog(tab="Dimensions", group="Window"));
  parameter Modelica.Units.SI.Length LX_window=2 "Window length"
    annotation (Dialog(tab="Dimensions", group="Window"));

  parameter Modelica.Units.SI.Mass m_cabin "Material mass within cabin"
    annotation (Dialog(group="Specifications"));
  parameter Modelica.Units.SI.SpecificHeatCapacity cp_cabin=900
    "Specific heat capacity of cabin material"
    annotation (Dialog(group="Specifications"));
  parameter Integer N_occupants[3] "Number of occupants: passengers, crew, pilots" annotation(Dialog(group="Specifications"));
  parameter Modelica.Units.SI.HeatFlowRate Q_int=750
    "Avionics heat generation";
  parameter Boolean fixed_Q = false "Use fixed heat and water input for occupants";
  parameter Modelica.Units.SI.HeatFlowRate Q_occupants=0
    "Total heat input from occupants" annotation (Dialog(enable=fixed_Q));
  parameter Modelica.Units.SI.MassFlowRate m_H20_occupants=0
    "Total water input from occupants" annotation (Dialog(enable=fixed_Q));
  //parameter Boolean use_E_fixed=false "Use the provided E_fixed value";
  //parameter Modelica.Units.SI.Irradiance E_fixed=900 "Fixed solar radiation" annotation (Dialog(enable=use_E_fixed));
  parameter Boolean use_r_eff=false "Use direct input for r_eff";
  parameter Real r_eff_di=0.02 "Reflection coefficient" annotation (Dialog(enable=use_r_eff));
  parameter Boolean use_fus_radiation=true "Use the component for fuselage radiation";

  parameter Modelica.Units.SI.MassFraction X_start=0.0120
    annotation (Dialog(group="Start Values"));
  parameter Modelica.Units.SI.Temperature T_start=28 + 273.15
    annotation (Dialog(group="Start Values"));

  final parameter Modelica.Units.SI.Length HZ_fus=HZ/2/sin(csi_fus)
    "Panel Z-length accounting for angle";

  constant Real pi = Modelica.Constants.pi;

  // Materials
  replaceable model Fuselage_int =
      ThermalManagement.Materials.AirbusEES.Fuselage                              constrainedby
    ThermalManagement.Materials.Properties
    "Internal fuselage material" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Materials"));
  replaceable model Fuselage_core =
      ThermalManagement.Materials.AirbusEES.Fuselage                               constrainedby
    ThermalManagement.Materials.Properties
    "Core fuselage material" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Materials"));
  replaceable model Fuselage_ext =
      ThermalManagement.Materials.AirbusEES.Fuselage                              constrainedby
    ThermalManagement.Materials.Properties
    "External fuselage material" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Materials"));
  // Heat correlations
  replaceable model HTC_int =
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    "Internal heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  replaceable model HTC_ext =
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    constrainedby
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    "External heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  replaceable model HTC_floor =
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    "Floor heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  replaceable model HTC_duct =
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    "Duct heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  replaceable model HTC_engine =
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    "Engine heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  replaceable model HTC_btp =
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    ThermalManagement.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    "Transmission heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  // Floor
  Components.HeatTransfer.WallConduction floorConduction(
    t=t_floor,
    A=A_floor,
    redeclare model Mat = Materials.AirbusEES.Floor,
    initOpt=ThermalManagement.Choices.InitOpt.fixedState,
    Tstart=313.15)
    annotation (Placement(transformation(extent={{-48,-40},{-28,-20}})));
  Components.HeatTransfer.InternalConvection floorIntConvection(A=A_floor,
      redeclare model HTC = HTC_int)   annotation (Placement(transformation(extent={{-48,-28},{-28,-8}})));
  Components.HeatTransfer.InternalConvection floorExtConvection(A=A_floor,
      redeclare model HTC = HTC_floor)   annotation (Placement(transformation(extent={{-48,-52},{-28,-32}})));
  BoundaryConditions.thermal floorExtTempInput(use_in_T=true, use_T=false)
    annotation (Placement(transformation(extent={{-44,-54},{-34,-48}})));
  // Fuselage
  Subsystems.UpperMachinery UpperMachinery(
    A_engine=A_engine,
    A_btp=A_btp,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext_btp = HTC_btp,
    redeclare model HTC_ext_engine = HTC_engine)
    annotation (Placement(transformation(extent={{-10,32},{10,52}})));
  Subsystems.FuselagePanel fuselagePanel_BL(
    csi=pi + csi_fus,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    redeclare model Fuselage_int = Fuselage_int,
    redeclare model Fuselage_core = Fuselage_core,
    redeclare model Fuselage_ext = Fuselage_ext,
    t=t_fus,
    W=HZ_fus,
    L=LX,
    include_window=false,
    T_start=T_start,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di,
    use_fus_radiation=use_fus_radiation)
    annotation (Placement(transformation(extent={{-34,-4},{-14,16}})));
  Subsystems.FuselagePanel fuselagePanel_TL(
    csi=csi_fus,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    redeclare model Fuselage_int = Fuselage_int,
    redeclare model Fuselage_core = Fuselage_core,
    redeclare model Fuselage_ext = Fuselage_ext,
    Nw_side=1,
    A_window=A_window,
    t_window=t_window,
    t=t_fus,
    W=HZ_fus,
    L=LX,
    include_window=true,
    T_start=T_start,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di,
    use_fus_radiation=use_fus_radiation)
          annotation (Placement(transformation(extent={{-34,16},{-14,36}})));
  Subsystems.FuselagePanel fuselagePanel_TR(
    csi=csi_fus,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    redeclare model Fuselage_int = Fuselage_int,
    redeclare model Fuselage_core = Fuselage_core,
    redeclare model Fuselage_ext = Fuselage_ext,
    Nw_side=1,
    A_window=A_window,
    t_window=t_window,
    t=t_fus,
    W=HZ_fus,
    L=LX,
    include_window=true,
    T_start=T_start,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di,
    use_fus_radiation=use_fus_radiation)
          annotation (Placement(transformation(extent={{32,16},{12,36}})));
  Subsystems.FuselagePanel fuselagePanel_BR(
    csi=pi + csi_fus,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    redeclare model Fuselage_int = Fuselage_int,
    redeclare model Fuselage_core = Fuselage_core,
    redeclare model Fuselage_ext = Fuselage_ext,
    t=t_fus,
    W=HZ_fus,
    L=LX,
    include_window=false,
    T_start=T_start,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di,
    use_fus_radiation=use_fus_radiation)
               annotation (Placement(transformation(extent={{32,-4},{12,16}})));
  Components.MassTransfer.Plenum plenum(
    V=LX*WY*HZ,
    noInitialTemperature=true,
    noInitialPressure=true,
    N_occupants=N_occupants,
    Q_int=Q_int,
    initOpt=environment.initOpt,
    T_start=T_start,
    X_start={X_start,1 - X_start},
    Q_sens_fixed=Q_occupants,
    m_H2O_fixed=m_H20_occupants,
    fixed_Q_sens=fixed_Q)
    annotation (Placement(transformation(extent={{-10,-82},{10,-62}})));
  // Ports
  CustomInterfaces.FluidPort_A inletPort
    annotation (Placement(transformation(extent={{-40,-94},{-20,-74}})));
  CustomInterfaces.FluidPort_B outletPort
    annotation (Placement(transformation(extent={{10,-94},{30,-74}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-52,-80},{-72,-60}})));
  Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{42,-82},{62,-62}})));
  Modelica.Blocks.Interfaces.RealOutput plenumTemp
    annotation (Placement(transformation(extent={{-100,30},{-120,50}})));
  Modelica.Blocks.Interfaces.RealOutput plenumPressure
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
  BoundaryConditions.radiation radiation_TR(
    use_theta=true,
    use_E=true,
    E=0,
    theta(displayUnit="rad") = 0)
                annotation (Placement(transformation(extent={{38,28},{44,32}})));
  BoundaryConditions.radiation radiation_BL(
    use_theta=true,
    use_E=true,
    theta(displayUnit="rad") = 0,
    E=0)
    annotation (Placement(transformation(extent={{-48,8},{-42,12}})));
  BoundaryConditions.radiation radiation_BR(
    use_theta=true,
    use_E=true,
    theta(displayUnit="rad") = 0,
    E=0)         annotation (Placement(transformation(extent={{38,8},{44,12}})));
  BoundaryConditions.radiation radiation_TL(
    use_theta=true,
    use_E=true,
    E=0,
    theta(displayUnit="rad") = 0)
    annotation (Placement(transformation(extent={{-52,22},{-44,28}})));
  // Duct
  Components.HeatTransfer.InternalConvection ductIntConvection(A=A_duct,
    redeclare model HTC = HTC_int) annotation (Placement(transformation(extent={{44,-30},{64,-10}})));
  Components.HeatTransfer.WallConduction ductConduction(
    initOpt=ThermalManagement.Choices.InitOpt.fixedState,
    redeclare model Mat = Materials.AirbusEES.Duct,
    t=t_duct,
    A=A_duct,
    Tstart=293.15)
    annotation (Placement(transformation(extent={{44,-42},{64,-22}})));
  Components.HeatTransfer.InternalConvection ductExtConduction(A=A_duct,
    redeclare model HTC = HTC_duct) annotation (Placement(transformation(extent={{44,-54},{64,-34}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a ductThermalPort
    annotation (Placement(transformation(extent={{96,-44},{116,-24}})));
  Modelica.Blocks.Interfaces.RealInput engineTempInput annotation (Placement(
        transformation(
        extent={{-15,-15},{15,15}},
        rotation=-90,
        origin={9,97})));
  Modelica.Blocks.Interfaces.RealInput transmissionTempInput annotation (
      Placement(transformation(
        extent={{-15,-15},{15,15}},
        rotation=-90,
        origin={51,97})));
  Modelica.Blocks.Interfaces.RealInput floorTempInput annotation (Placement(
        transformation(
        extent={{-15,-15},{15,15}},
        rotation=-90,
        origin={-53,99})));

equation

  connect(floorIntConvection.outlet, floorConduction.inlet)
    annotation (Line(points={{-38,-21.4},{-38,-26.6}},
                                                     color={191,0,0}));
  connect(floorExtConvection.inlet, floorConduction.outlet)
    annotation (Line(points={{-38,-38.6},{-38,-33.4}},
                                                     color={191,0,0}));
  connect(floorExtTempInput.thermal, floorExtConvection.outlet)
    annotation (Line(points={{-37.3333,-51},{-37.3333,-48},{-38,-48},{-38,
          -45.4}},                               color={191,0,0}));
  connect(outletPort, outletPort)
    annotation (Line(points={{20,-84},{20,-84}},          color={0,0,0}));
  connect(outletPort, plenum.outlet)
    annotation (Line(points={{20,-84},{20,-72},{10,-72}}, color={0,0,0}));
  connect(inletPort, plenum.inlet)
    annotation (Line(points={{-30,-84},{-30,-72},{-10,-72}},
                                                           color={0,0,0}));
  connect(pressureSensor.port, plenum.outlet)
    annotation (Line(points={{42,-72},{10,-72}}, color={0,0,0}));
  connect(pressureSensor.y, plenumPressure)
    annotation (Line(points={{63,-72},{80,-72},{80,40},{110,40}},
                                                 color={0,0,127}));
  connect(inletPort, inletPort) annotation (Line(points={{-30,-84},{-30,-84}},
                                color={0,0,0}));
  connect(temperatureSensor.T, plenumTemp)
    annotation (Line(points={{-73,-70},{-80,-70},{-80,40},{-110,40}},
                                                   color={0,0,127}));
  connect(ductConduction.inlet, ductIntConvection.outlet)
    annotation (Line(points={{54,-28.6},{54,-23.4}}, color={191,0,0}));
  connect(ductExtConduction.inlet, ductConduction.outlet)
    annotation (Line(points={{54,-40.6},{54,-35.4}}, color={191,0,0}));
  connect(ductThermalPort, ductExtConduction.outlet) annotation (Line(
        points={{106,-34},{106,-47.4},{54,-47.4}},
                                                 color={191,0,0}));
  connect(ductThermalPort, ductThermalPort) annotation (Line(points={{106,-34},{
          106,-34}},               color={191,0,0}));
  connect(radiation_BL.radiative,fuselagePanel_BL. irradiancePort)
    annotation (Line(points={{-44,10},{-33,10},{-33,6}},  color={191,0,0}));
  connect(radiation_BR.radiative,fuselagePanel_BR. irradiancePort) annotation (
      Line(points={{42,10},{36,10},{36,6},{31,6}},   color={191,0,0}));
  connect(fuselagePanel_TL.thermalPortInner, fuselagePanel_BR.thermalPortInner)
    annotation (Line(points={{-15,29},{0,29},{0,9},{13,9}}, color={191,0,0}));
  connect(fuselagePanel_TR.thermalPortInner, fuselagePanel_BR.thermalPortInner)
    annotation (Line(points={{13,29},{0,29},{0,9},{13,9}}, color={191,0,0}));
  connect(radiation_TL.radiative,fuselagePanel_TL. irradiancePort)
    annotation (Line(points={{-46.6667,25},{-44,25},{-44,26},{-33,26}},
                                                 color={191,0,0}));
  connect(fuselagePanel_TR.thermalPortInner, UpperMachinery.outlet)
    annotation (Line(points={{13,29},{0,29},{0,39.8}}, color={191,0,0}));
  connect(fuselagePanel_BL.thermalPortInner, fuselagePanel_BR.thermalPortInner)
    annotation (Line(points={{-15,9},{13,9}}, color={191,0,0}));
  connect(fuselagePanel_TR.irradiancePort, radiation_TR.radiative)
    annotation (Line(points={{31,26},{42,26},{42,30}}, color={191,0,0}));
  connect(plenum.thermalPort, fuselagePanel_BR.thermalPortInner)
    annotation (Line(points={{0,-63},{0,9},{13,9}}, color={191,0,0}));
  connect(ductIntConvection.inlet, fuselagePanel_BR.thermalPortInner)
    annotation (Line(points={{54,-16.6},{54,-8},{0,-8},{0,9},{13,9}}, color={191,
          0,0}));
  connect(temperatureSensor.port, plenum.thermalPort) annotation (Line(points={{-52,-70},
          {-42,-70},{-42,-60},{0,-60},{0,-63}},          color={191,0,0}));
  connect(floorIntConvection.inlet, fuselagePanel_BR.thermalPortInner)
    annotation (Line(points={{-38,-14.6},{-38,-8},{0,-8},{0,9},{13,9}}, color={191,
          0,0}));
  connect(transmissionTempInput, UpperMachinery.transmissionTempInput)
    annotation (Line(points={{51,97},{51,51.7},{0.5,51.7}}, color={0,0,127}));
  connect(UpperMachinery.engineTempInput, engineTempInput) annotation (Line(
        points={{-4.5,51.7},{-4,51.7},{-4,72},{9,72},{9,97}},
                                                  color={0,0,127}));
  connect(floorTempInput, floorExtTempInput.in_T)
    annotation (Line(points={{-53,99},{-53,-48},{-45,-48}}, color={0,0,127}));
  connect(fuselagePanel_TL.thermalPortToSeats, fuselagePanel_BR.thermalPortInner)
    annotation (Line(points={{-15,23},{0,23},{0,9},{13,9}}, color={191,0,0}));
  connect(fuselagePanel_TR.thermalPortToSeats, fuselagePanel_BR.thermalPortInner)
    annotation (Line(points={{13,23},{0,23},{0,9},{13,9}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Polygon(
          points={{-50,100},{24,100},{52,100},{100,20},{60,-80},{-60,-80},
              {-100,20},{-50,100}},
          lineColor={28,108,200},
          fillColor={85,170,255},
          fillPattern=FillPattern.Backward)}),                   Diagram(
        coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-58,-12},{-18,-54}},
                                              lineColor={28,108,200}),
        Text(
          extent={{-58,-8},{-36,-12}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Floor"),
        Rectangle(extent={{40,-14},{70,-52}}, lineColor={28,108,200}),
        Text(
          extent={{36,-10},{58,-14}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Duct")}));
end HeliCabin;
