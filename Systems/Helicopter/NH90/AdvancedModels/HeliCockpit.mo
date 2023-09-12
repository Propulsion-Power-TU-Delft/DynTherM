within DynTherM.Systems.Helicopter.NH90.AdvancedModels;
model HeliCockpit
  "Helicopter cockpit advanced model made to match with basic NH90 Airbus"
  outer Components.Environment environment;
  parameter Modelica.Units.SI.Length LX=0.446 "Fuselage X-length" annotation (Dialog(tab="Dimensions", group="General"));
  parameter Modelica.Units.SI.Length WY=2.6 "Fuselage Y-length" annotation (Dialog(tab="Dimensions", group="General"));
  parameter Modelica.Units.SI.Length HZ=2.8 "Fuselage Z-length" annotation (Dialog(tab="Dimensions", group="General"));

  parameter Modelica.Units.SI.Length t_fus=0.03 "Fuselage wall thickness" annotation (Dialog(tab="Dimensions", group="Fuselage"));
  parameter Modelica.Units.SI.Area A_floor=2.5 "Floor surface area";
  parameter Modelica.Units.SI.Length t_floor=0.025 "Floor thickness" annotation (Dialog(tab="Dimensions", group="Fuselage"));
  parameter Modelica.Units.SI.Area A_duct=2 "Duct surface area" annotation (Dialog(tab="Dimensions", group="Fuselage"));
  parameter Modelica.Units.SI.Length t_duct=0.0004785 "Duct thickness" annotation (Dialog(tab="Dimensions", group="Fuselage"));
  parameter Modelica.Units.SI.Angle csi_fus=90*pi/180
    "Angle of panels (acute)" annotation (Dialog(tab="Dimensions", group="Fuselage"));

  parameter Modelica.Units.SI.Area A_window=0 "Window area" annotation (Dialog(tab="Dimensions", group="Window"));
  parameter Modelica.Units.SI.Area Ap_window=0 "Window projected area" annotation (Dialog(tab="Dimensions", group="Window"));
  parameter Modelica.Units.SI.Length t_window=0.01 "Window thickness" annotation (Dialog(tab="Dimensions", group="Window"));
  parameter Modelica.Units.SI.Length LX_window=1 "Window X-length" annotation (Dialog(tab="Dimensions", group="Window"));

  parameter Modelica.Units.SI.Length HZ_ws=2 "Windscreen total Z-length" annotation (Dialog(tab="Dimensions", group="Windscreen"));
  parameter Modelica.Units.SI.Length WY_ws=2.15 "Windscreen total Y-length" annotation (Dialog(tab="Dimensions", group="Windscreen"));
  parameter Modelica.Units.SI.Area A_ws=4.3 "Windscreen area" annotation (Dialog(tab="Dimensions", group="Windscreen"));
  parameter Modelica.Units.SI.Area Ap_ws=3.5 "Windscreen projected area" annotation (Dialog(tab="Dimensions", group="Windscreen"));
  parameter Modelica.Units.SI.Length t_ws=0.03 "Windscreen thickness" annotation (Dialog(tab="Dimensions", group="Windscreen"));
  parameter Modelica.Units.SI.Angle csi_ws=90*pi/180
    "Angle of windscreen (acute)" annotation (Dialog(tab="Dimensions", group="Windscreen"));

  parameter Modelica.Units.SI.Mass m_cockpit "Material mass within cockpit" annotation (Dialog(group="Specifications"));
  parameter Modelica.Units.SI.SpecificHeatCapacity cp_cockpit=900
    "Specific heat capacity of cockpit material" annotation (Dialog(group="Specifications"));
  parameter Integer N_occupants[3]  "Number of occupants: passengers, crew, pilots" annotation(Dialog(group="Specifications"));
  parameter Modelica.Units.SI.HeatFlowRate Q_int=1000
    "Avionics heat generation";
  parameter Boolean fixed_Q = false "Use fixed heat and water input for occupants";
  parameter Modelica.Units.SI.HeatFlowRate Q_occupants[3]={0,0,0}
    "Total heat input from occupants" annotation (Dialog(enable=fixed_Q));
  parameter Modelica.Units.SI.MassFlowRate m_H20_occupants[3]={0,0,0}
    "Total water input from occupants" annotation (Dialog(enable=fixed_Q));
  //parameter Boolean use_E_fixed=false "Use the provided E_fixed value";
  //parameter Boolean use_E_fixed_ws=false "Use the provided E_fixed_ws value";
  //parameter Modelica.Units.SI.Irradiance E_fixed=900 "Fixed solar radiation" annotation (Dialog(enable=use_E_fixed));
  //parameter Modelica.Units.SI.Irradiance E_fixed_ws=900 "Fixed solar radiation windscreen" annotation (Dialog(enable=use_E_fixed_ws));
  parameter Boolean use_r_eff=false "Use direct input for r_eff";
  parameter Real r_eff_di=0.02 "Reflection coefficient" annotation (Dialog(enable=use_r_eff));
  parameter Real r_eff_di_ws=0.02 "Reflection coefficient of windscreen" annotation (Dialog(enable=use_r_eff));
  parameter Boolean use_fus_radiation=true "Use the component for fuselage radiation";

  parameter Modelica.Units.SI.MassFraction X_start=0.00894 annotation (Dialog(group="Start Values"));
  parameter Modelica.Units.SI.Temperature T_start=301.15 annotation (Dialog(group="Start Values"));

  final parameter Modelica.Units.SI.Length HZ_fus=HZ/2/sin(csi_fus)
    "diagonal height per panel";
  final parameter Modelica.Units.SI.Length HZ_ws_angled=HZ_ws/sin(csi_ws);
  constant Real pi = Modelica.Constants.pi;

  // Materials
  replaceable model Fuselage_int =
      DynTherM.Materials.AirbusEES.Fuselage constrainedby
    DynTherM.Materials.Properties
    "Internal fuselage material" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Materials"));
  replaceable model Fuselage_core =
      DynTherM.Materials.AirbusEES.Fuselage constrainedby
    DynTherM.Materials.Properties
    "Core fuselage material" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Materials"));
  replaceable model Fuselage_ext =
      DynTherM.Materials.AirbusEES.Fuselage constrainedby
    DynTherM.Materials.Properties
    "External fuselage material" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Materials"));
  // Heat correlations
  replaceable model HTC_int =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    "Internal heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  replaceable model HTC_ext =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    "External heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  replaceable model HTC_ext_ws =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    "External windscreen heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  replaceable model HTC_floor =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    "Floor heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  replaceable model HTC_duct =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    "Duct heat correlation" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));

  // Floor
  Components.HeatTransfer.WallConduction floorConduction(
    t=t_floor,
    A=A_floor,
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    redeclare model Mat = Materials.AirbusEES.Floor,
    Tstart=323.15)
    annotation (Placement(transformation(extent={{-44,-30},{-24,-10}})));
  Components.HeatTransfer.InternalConvection floorIntConvection(A=A_floor,
      redeclare model HTC = HTC_int)                             annotation (Placement(transformation(extent={{-44,-18},
            {-24,2}})));
  Components.HeatTransfer.InternalConvection floorExtConvection(A=A_floor,
      redeclare model HTC = HTC_floor)                          annotation (Placement(transformation(extent={{-44,-42},
            {-24,-22}})));
  BoundaryConditions.thermal floorExtTempInput(use_T=false, use_in_T=true)
    annotation (Placement(transformation(extent={{-38,-42},{-32,-38}})));
  // Windscreen
  // Fuselage
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
    fixed_Q=false)
    annotation (Placement(transformation(extent={{-10,-80},{10,-60}})));
  Subsystems.FuselagePanel fuselagePanel_TL(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    redeclare model Fuselage_int = Fuselage_int,
    redeclare model Fuselage_core = Fuselage_core,
    redeclare model Fuselage_ext = Fuselage_ext,
    Nw_side=1,
    A_window=A_window,
    t_window=t_window,
    t=t_fus,
    csi=csi_fus,
    W=HZ_fus,
    L=LX,
    T_start=T_start,
    include_window=false,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di,
    use_fus_radiation=use_fus_radiation)
    annotation (Placement(transformation(extent={{-26,18},{-6,38}})));
  Subsystems.FuselagePanel fuselagePanel_BL(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    redeclare model Fuselage_int = Fuselage_int,
    redeclare model Fuselage_core = Fuselage_core,
    redeclare model Fuselage_ext = Fuselage_ext,
    t=t_fus,
    W=HZ_fus,
    L=LX,
    csi=pi + csi_fus,
    include_window=false,
    T_start=T_start,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di,
    use_fus_radiation=use_fus_radiation)
    annotation (Placement(transformation(extent={{-26,0},{-6,20}})));
  Subsystems.FuselagePanel fuselagePanel_TR(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    redeclare model Fuselage_int = Fuselage_int,
    redeclare model Fuselage_core = Fuselage_core,
    redeclare model Fuselage_ext = Fuselage_ext,
    Nw_side=1,
    A_window=A_window,
    t_window=t_window,
    t=t_fus,
    csi=csi_fus,
    W=HZ_fus,
    L=LX,
    T_start=T_start,
    include_window=false,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di,
    use_fus_radiation=use_fus_radiation)
    annotation (Placement(transformation(extent={{26,18},{6,38}})));
  Subsystems.FuselagePanel fuselagePanel_BR(
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext,
    redeclare model Fuselage_int = Fuselage_int,
    redeclare model Fuselage_core = Fuselage_core,
    redeclare model Fuselage_ext = Fuselage_ext,
    t=t_fus,
    W=HZ_fus,
    L=LX,
    csi=pi + csi_fus,
    include_window=false,
    T_start=T_start,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di,
    use_fus_radiation=use_fus_radiation)
    annotation (Placement(transformation(extent={{26,0},{6,20}})));
  // Duct
  Components.HeatTransfer.InternalConvection ductIntConvection(A=A_duct,
    redeclare model HTC = HTC_int) annotation (Placement(transformation(extent={{48,-24},{68,-4}})));
  Components.HeatTransfer.WallConduction ductConduction(
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    redeclare model Mat = Materials.AirbusEES.Duct,
    t=t_duct,
    A=A_duct,
    Tstart=323.15)
    annotation (Placement(transformation(extent={{48,-36},{68,-16}})));
  Components.HeatTransfer.InternalConvection ductExtConduction(A=A_duct,
    redeclare model HTC = HTC_duct) annotation (Placement(transformation(extent={{48,-48},{68,-28}})));
    // Ports
  CustomInterfaces.FluidPort_A inletPort
    annotation (Placement(transformation(extent={{-30,-96},{-10,-76}})));
  CustomInterfaces.FluidPort_B outletPort
    annotation (Placement(transformation(extent={{10,-96},{30,-76}})));

  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-50,-80},{-70,-60}})));
  Sensors.PressureSensor pressureSensor
    annotation (Placement(transformation(extent={{50,-80},{70,-60}})));
  Modelica.Blocks.Interfaces.RealOutput plenumTemp annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-110,40})));
  Modelica.Blocks.Interfaces.RealOutput plenumPressure
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a ductThermalPort
    annotation (Placement(transformation(extent={{84,-62},{104,-42}})));
  BoundaryConditions.radiation radiation_TL(
    use_E=true,
    use_theta=true,
    theta=0,
    E=0)
    annotation (Placement(transformation(extent={{-42,24},{-34,30}})));
  BoundaryConditions.radiation radiation_TR(
    use_theta=true,
    theta=0,
    use_E=true,
    E=0)        annotation (Placement(transformation(extent={{36,26},{40,30}})));
  BoundaryConditions.radiation radiation_BL(
    theta=0,
    use_theta=true,
    use_E=true,
    E=0)
    annotation (Placement(transformation(extent={{-42,8},{-34,12}})));
  BoundaryConditions.radiation radiation_BR(
    use_theta=true,
    theta=0,
    use_E=true,
    E=0)    annotation (Placement(transformation(extent={{34,8},{40,12}})));
  Modelica.Blocks.Interfaces.RealInput floorTempInput annotation (Placement(
        transformation(
        extent={{-15,-15},{15,15}},
        rotation=-90,
        origin={-57,99})));
  Subsystems.Window windscreen_R(
    L=HZ_ws_angled,
    W=WY_ws/2,
    csi=csi_ws,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext_ws,
    t=t_ws,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di_ws)
                annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={14,62})));
   BoundaryConditions.radiation radiation_WSR(
    use_theta=true,
    use_E=true,
    theta=0,
    E=900)  annotation (Placement(transformation(extent={{8,88},{14,92}})));
   BoundaryConditions.radiation radiation_WSL(
    use_theta=true,
    use_E=true,
    theta=0,
    E=900)
    annotation (Placement(transformation(extent={{-14,88},{-8,92}})));
  Subsystems.Window windscreen_L(
    L=HZ_ws_angled,
    W=WY_ws/2,
    csi=csi_ws,
    redeclare model HTC_int = HTC_int,
    redeclare model HTC_ext = HTC_ext_ws,
    t=t_ws,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di_ws)
                annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-8,62})));

equation

  connect(floorIntConvection.outlet, floorConduction.inlet)
    annotation (Line(points={{-34,-11.4},{-34,-16.6}},
                                                   color={191,0,0}));
  connect(floorConduction.outlet, floorExtConvection.inlet)
    annotation (Line(points={{-34,-23.4},{-34,-28.6}},
                                                  color={191,0,0}));
  connect(floorExtTempInput.thermal, floorExtConvection.outlet)
    annotation (Line(points={{-34,-40},{-34,-35.4}},
                                                 color={191,0,0}));
  connect(outletPort, outletPort)
    annotation (Line(points={{20,-86},{20,-86}}, color={0,0,0}));
  connect(outletPort, plenum.outlet)
    annotation (Line(points={{20,-86},{20,-70},{10,-70}}, color={0,0,0}));
  connect(inletPort, inletPort)
    annotation (Line(points={{-20,-86},{-20,-86}}, color={0,0,0}));
  connect(plenum.inlet, inletPort) annotation (Line(points={{-10,-70},{-20,-70},
          {-20,-86}},      color={0,0,0}));
  connect(plenum.thermalPort, temperatureSensor.port) annotation (Line(
        points={{0,-61},{-24,-61},{-24,-70},{-50,-70}}, color={191,0,0}));
  connect(pressureSensor.port, plenum.outlet)
    annotation (Line(points={{50,-70},{10,-70}}, color={0,0,0}));
  connect(pressureSensor.y, plenumPressure)
    annotation (Line(points={{71,-70},{80,-70},{80,40},{110,40}},
                                                 color={0,0,127}));
  connect(plenumTemp, plenumTemp)
    annotation (Line(points={{-110,40},{-110,40}}, color={0,0,127}));
  connect(temperatureSensor.T, plenumTemp)
    annotation (Line(points={{-71,-70},{-80,-70},{-80,40},{-110,40}},
                                                   color={0,0,127}));
  connect(ductConduction.inlet,ductIntConvection. outlet)
    annotation (Line(points={{58,-22.6},{58,-17.4}}, color={191,0,0}));
  connect(ductExtConduction.inlet,ductConduction. outlet)
    annotation (Line(points={{58,-34.6},{58,-29.4}}, color={191,0,0}));
  connect(ductExtConduction.outlet, ductThermalPort) annotation (Line(points={{58,
          -41.4},{58,-52},{94,-52}},                    color={191,0,0}));
  connect(ductIntConvection.inlet, plenum.thermalPort) annotation (Line(points={
          {58,-10.6},{58,0},{0,0},{0,-61}}, color={191,0,0}));
  connect(floorIntConvection.inlet, plenum.thermalPort) annotation (Line(points={{-34,
          -4.6},{-34,0},{0,0},{0,-61}},      color={191,0,0}));
  connect(fuselagePanel_BR.thermalPortInner, plenum.thermalPort)
    annotation (Line(points={{7,13},{0,13},{0,-61}}, color={191,0,0}));
  connect(fuselagePanel_BL.thermalPortInner, plenum.thermalPort)
    annotation (Line(points={{-7,13},{0,13},{0,-61}}, color={191,0,0}));
  connect(fuselagePanel_TL.thermalPortInner, plenum.thermalPort)
    annotation (Line(points={{-7,31},{0,31},{0,-61}}, color={191,0,0}));
  connect(fuselagePanel_TR.thermalPortInner, plenum.thermalPort)
    annotation (Line(points={{7,31},{0,31},{0,-61}}, color={191,0,0}));
  connect(radiation_TR.radiative, fuselagePanel_TR.irradiancePort)
    annotation (Line(points={{38.6667,28},{25,28}},
                                               color={191,0,0}));
  connect(radiation_BL.radiative, fuselagePanel_BL.irradiancePort)
    annotation (Line(points={{-36.6667,10},{-25,10}},
                                                 color={191,0,0}));
  connect(radiation_BR.radiative, fuselagePanel_BR.irradiancePort)
    annotation (Line(points={{38,10},{25,10}}, color={191,0,0}));
  connect(radiation_TL.radiative, fuselagePanel_TL.irradiancePort) annotation (
      Line(points={{-36.6667,27},{-31.3333,27},{-31.3333,28},{-25,28}}, color={191,
          0,0}));
  connect(floorTempInput, floorExtTempInput.in_T) annotation (Line(points={{-57,99},
          {-57,-38},{-38.6,-38}},     color={0,0,127}));
  connect(windscreen_R.irradiancePort,radiation_WSR. radiative) annotation (
      Line(points={{13,71},{13,85.5},{12,85.5},{12,90}}, color={191,0,0}));
  connect(windscreen_L.irradiancePort, radiation_WSL.radiative)
    annotation (Line(points={{-7,71},{-7,90},{-10,90}}, color={191,0,0}));
  connect(windscreen_R.thermalPortInner, plenum.thermalPort)
    annotation (Line(points={{10.4,53},{0,53},{0,-61}}, color={191,0,0}));
  connect(windscreen_L.thermalPortInner, plenum.thermalPort)
    annotation (Line(points={{-4.4,53},{0,53},{0,-61}}, color={191,0,0}));
  connect(windscreen_L.thermalPortToSeats, plenum.thermalPort) annotation (Line(
        points={{-11,53},{-10,53},{-10,42},{0,42},{0,-61}}, color={191,0,0}));
  connect(windscreen_R.thermalPortToSeats, plenum.thermalPort) annotation (Line(
        points={{17,53},{17,42},{0,42},{0,-61}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{38,-80},{-102,-2}},
          lineColor={28,108,200},
          startAngle=0,
          endAngle=360,
          fillColor={85,170,255},
          fillPattern=FillPattern.Backward),
          Polygon(
          points={{-40,100},{24,100},{52,100},{100,20},{60,-80},{-34,-80},
              {6,40},{-40,100}},
          lineColor={28,108,200},
          fillColor={85,170,255},
          fillPattern=FillPattern.Backward),
        Polygon(
          points={{-80,-12},{-40,100},{-40,100},{-26,82},{-52,-4},{-80,
              -12}},
          lineColor={28,108,200},
          lineThickness=1)}),                                    Diagram(
        coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-52,-4},{-14,-42}},lineColor={28,108,200}),
        Text(
          extent={{-56,0},{-34,-4}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Floor"),
        Rectangle(extent={{44,-8},{76,-48}},  lineColor={28,108,200}),
        Text(
          extent={{40,-4},{62,-8}},
          lineColor={28,108,200},
          fontSize=10,
          textString="Duct")}));
end HeliCockpit;
