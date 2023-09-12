within DynTherM.Examples.Helicopter;
model NH90Ref
  "NH90 with reference Airbus provided values for comparison to the EES model"
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_int_fixed=15
    "Internal convection fixed heat transfer coefficient"
    annotation (Dialog(group="Heat Correlation"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_ext_ck_fixed=80
    "External cockpit convection fixed heat transfer coefficient"
    annotation (Dialog(group="Heat Correlation"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_ext_cb_fixed=100
    "External cabin convection fixed heat transfer coefficient"
    annotation (Dialog(group="Heat Correlation"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_ext_ws_fixed=80
    "External windscreen convection fixed heat transfer coefficient"
    annotation (Dialog(group="Heat Correlation"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_floor_fixed=25
    "Floor internal convection fixed heat transfer coefficient"
    annotation (Dialog(group="Heat Correlation"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_duct_fixed=80
    "Duct internal convection fixed heat transfer coefficient"
    annotation (Dialog(group="Heat Correlation"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_engine_fixed=25
    "Engine internal convection fixed heat transfer coefficient"
    annotation (Dialog(group="Heat Correlation"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_btp_fixed=25
    "Transmission internal convection fixed heat transfer coefficient"
    annotation (Dialog(group="Heat Correlation"));
  parameter Integer Npax=20 "Number of passengers";
  parameter Modelica.Units.SI.Temperature T_ck=301.15
    "Cockpit target temperature" annotation (Dialog(group="Temperature"));
  parameter Modelica.Units.SI.Temperature T_cb=301.15
    "Cabin target temperature" annotation (Dialog(group="Temperature"));
  parameter Modelica.Units.SI.Temperature T_floor=323.15
    "Floor temperature" annotation (Dialog(group="Temperature"));
  parameter Modelica.Units.SI.Temperature T_engine=373.15
    "Engine temperature" annotation (Dialog(group="Temperature"));
  parameter Modelica.Units.SI.Temperature T_btp=353.15
    "Transmission temperature" annotation (Dialog(group="Temperature"));
  parameter Modelica.Units.SI.MassFraction X_env = 0.01877 "Ambient Absolute water humidity";

  Modelica.Blocks.Sources.Constant engineTemp(k=T_engine)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-6,80})));
  Modelica.Blocks.Sources.Constant transmissionTemp(k=T_btp)
    annotation (Placement(transformation(extent={{56,72},{36,92}})));
  Modelica.Blocks.Sources.Constant floorTemp(k=T_floor)
    annotation (Placement(transformation(extent={{-64,52},{-44,72}})));
  Modelica.Blocks.Sources.Constant targetTempCabin(k=T_cb)
    annotation (Placement(transformation(extent={{-10,-82},{10,-62}})));
  Modelica.Blocks.Sources.Constant targetTempCockpit(k=T_ck)
    annotation (Placement(transformation(extent={{-56,-80},{-36,-60}})));
  Systems.Helicopter.NH90.AdvancedModels.HeliCombinedRef heliCombined(
    use_E_fixed=true,
    X_env=X_env,
    redeclare model HTC_int =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_int_fixed),
    redeclare model HTC_ext_ck =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue (
          ht_fixed=ht_ext_ck_fixed),
    redeclare model HTC_ext_cb =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue (
          ht_fixed=ht_ext_cb_fixed),
    redeclare model HTC_floor =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_floor_fixed),
    redeclare model HTC_duct =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_duct_fixed),
    redeclare model HTC_engine =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_engine_fixed),
    redeclare model HTC_btp =
        Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue (
          ht_fixed=ht_btp_fixed),
    T_cb(displayUnit="degC"),
    redeclare model HTC_ext_ws =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue (
          ht_fixed=ht_ext_ws_fixed),
    N_occupants={Npax,0,2},
    E_fixed=0,
    E_fixed_ws=900)
    annotation (Placement(transformation(extent={{-50,-32},{34,36}})));
  inner Components.Environment environment,
    P_amb_di(displayUnit="bar"),
    T_amb_di=313.15,
    use_T_amb=true, Error,
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    Altitude=0,
    phi_amb=0.1,
    T_ground=313.15,
    use_P_amb=true;
equation
  connect(floorTemp.y, heliCombined.floorTempCockpitInput) annotation (Line(
        points={{-43,62},{-35.3,62},{-35.3,35.66}}, color={0,0,127}));
  connect(transmissionTemp.y, heliCombined.transmissionTempInput) annotation (
      Line(points={{35,82},{20.14,82},{20.14,34.98}}, color={0,0,127}));
  connect(engineTemp.y, heliCombined.engineTempInput) annotation (Line(points={{5,80},{
          10.06,80},{10.06,34.98}},         color={0,0,127}));
  connect(heliCombined.floorTempCabinInput, heliCombined.floorTempCockpitInput)
    annotation (Line(points={{-10.1,34.98},{-10.1,62},{-35.3,62},{-35.3,35.66}},
        color={0,0,127}));
  connect(targetTempCabin.y, heliCombined.targetTempCabinInput) annotation (
      Line(points={{11,-72},{12.58,-72},{12.58,-24.86}}, color={0,0,127}));
  connect(heliCombined.targetTempCockpitInput, targetTempCockpit.y) annotation (
     Line(points={{-29.42,-24.86},{-29.42,-70},{-35,-70}}, color={0,0,127}));
    annotation (Placement(transformation(extent={{-96,-12},{-76,8}})),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=500));
end NH90Ref;
