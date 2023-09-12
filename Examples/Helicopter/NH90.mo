within DynTherM.Examples.Helicopter;
model NH90 "NH90 running model"
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

  inner Components.Environment environment(
    ISA_plus=0,
    P_amb_di(displayUnit="bar"),
    use_T_amb=true,
    use_ext_sw=true,
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    Altitude=2000,
    phi_amb=0.4,
    T_ground=313.15,
    use_P_amb=true)
    annotation (Placement(transformation(extent={{70,70},{100,100}})));
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
  Systems.Helicopter.NH90.AdvancedModels.HeliCombined heliCombined(
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
    N_occupants={Npax,0,2})
    annotation (Placement(transformation(extent={{-44,-30},{44,34}})));
equation
  connect(transmissionTemp.y, heliCombined.transmissionTempInput) annotation (
      Line(points={{35,82},{29.48,82},{29.48,33.04}}, color={0,0,127}));
  connect(engineTemp.y, heliCombined.engineTempInput) annotation (Line(points={
          {5,80},{18.92,80},{18.92,33.04}}, color={0,0,127}));
  connect(floorTemp.y, heliCombined.floorTempCockpitInput) annotation (Line(
        points={{-43,62},{-28.6,62},{-28.6,33.68}}, color={0,0,127}));
  connect(heliCombined.floorTempCabinInput, heliCombined.floorTempCockpitInput)
    annotation (Line(points={{-2.2,33.04},{-2.2,62},{-28.6,62},{-28.6,33.68}},
        color={0,0,127}));
  connect(targetTempCockpit.y, heliCombined.targetTempCockpitInput) annotation (
     Line(points={{-35,-70},{-27.72,-70},{-27.72,-26.48}}, color={0,0,127}));
  connect(targetTempCabin.y, heliCombined.targetTempCabinInput) annotation (
      Line(points={{11,-72},{14.52,-72},{14.52,-23.92}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=2000, __Dymola_NumberOfIntervals=1000));
end NH90;
