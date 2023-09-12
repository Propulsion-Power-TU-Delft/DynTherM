within DynTherM.Systems.Helicopter.Subsystems;
model UpperMachinery
  "Represents engien and transmission component on upper section."
  outer Components.Environment environment;
  parameter Modelica.Units.SI.Temperature T_start=400 "Start temperature";
  parameter Modelica.Units.SI.Area A_engine=4.2 "Engine wall surface area" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Area A_btp=4.2 "Transmission wall surface area" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_engine=0.04 "Engine wall thickness" annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t_btp=0.04 "Exhaust wall thickness" annotation (Dialog(group="Dimensions"));

  replaceable model HTC_int =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal annotation (choicesAllMatching=true);

  replaceable model HTC_ext_btp =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal annotation (choicesAllMatching=true);

  replaceable model HTC_ext_engine =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal annotation (choicesAllMatching=true);

  // Engine
  Components.HeatTransfer.InternalConvection engineIntConvection(A=A_engine,
    redeclare model HTC = HTC_int) annotation (Placement(transformation(extent={{-50,16},{-30,36}})));
  Components.HeatTransfer.WallConduction engineConduction(
    redeclare model Mat = Materials.AirbusEES.EngineAluminium,
    A=A_engine,
    t=t_engine,
    Tstart=T_start,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-50,28},{-30,48}})));
  BoundaryConditions.thermal engineTemp(
    use_T=false,
    use_in_T=true)
    annotation (Placement(transformation(extent={{-46,60},{-36,68}})));
  Components.HeatTransfer.InternalConvection engineExtConvection(A=A_engine,
    redeclare model HTC = HTC_ext_engine) annotation (Placement(transformation(extent={{-50,42},{-30,62}})));
  // Transmission
  Components.HeatTransfer.InternalConvection transmissionExtConvection(A=A_btp,
    redeclare model HTC = HTC_ext_btp) annotation (Placement(transformation(extent={{6,40},{26,60}})));
  Components.HeatTransfer.WallConduction TransmissionWall(
    redeclare model Mat = Materials.AirbusEES.TransmissionAluminium,
    A=A_btp,
    t=t_btp,
    Tstart=T_start,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{6,26},{26,46}})));
  BoundaryConditions.thermal transmissionTemp(
    use_T=false,
    use_in_T=true)
    annotation (Placement(transformation(extent={{8,62},{20,70}})));
  Components.HeatTransfer.InternalConvection transmissionIntConvection(
      redeclare model HTC = HTC_int, A=A_btp) annotation (Placement(transformation(extent={{6,12},{26,32}})));
  // Ports
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet
    annotation (Placement(transformation(extent={{-10,-32},{10,-12}}),
        iconTransformation(extent={{-10,-32},{10,-12}})));
  Modelica.Blocks.Interfaces.RealInput engineTempInput annotation (Placement(
        transformation(
        extent={{-15,-15},{15,15}},
        rotation=-90,
        origin={-45,97})));
  Modelica.Blocks.Interfaces.RealInput transmissionTempInput annotation (
      Placement(transformation(
        extent={{-15,-15},{15,15}},
        rotation=-90,
        origin={5,97})));
equation
  connect(TransmissionWall.outlet, transmissionIntConvection.inlet)
    annotation (Line(points={{16,32.6},{16,25.4}}, color={191,0,0}));
  connect(transmissionIntConvection.outlet, outlet)
    annotation (Line(points={{16,18.6},{16,-22},{0,-22}}, color={191,0,0}));
  connect(engineConduction.outlet, engineIntConvection.inlet)
    annotation (Line(points={{-40,34.6},{-40,29.4}}, color={191,0,0}));
  connect(transmissionExtConvection.outlet, TransmissionWall.inlet)
    annotation (Line(points={{16,46.6},{16,39.4}}, color={191,0,0}));
  connect(transmissionExtConvection.inlet, transmissionTemp.thermal)
    annotation (Line(points={{16,53.4},{18,53.4},{18,66},{16,66}}, color={191,0,
          0}));
  connect(engineExtConvection.outlet, engineConduction.inlet)
    annotation (Line(points={{-40,48.6},{-40,41.4}}, color={191,0,0}));
  connect(engineTemp.thermal, engineExtConvection.inlet) annotation (Line(
        points={{-39.3333,64},{-40,64},{-40,55.4}}, color={191,0,0}));
  connect(engineIntConvection.outlet, outlet)
    annotation (Line(points={{-40,22.6},{-40,-22},{0,-22}}, color={191,0,0}));
  connect(engineTempInput, engineTemp.in_T)
    annotation (Line(points={{-45,97},{-45,68},{-47,68}}, color={0,0,127}));
  connect(transmissionTempInput, transmissionTemp.in_T)
    annotation (Line(points={{5,97},{5,70},{6.8,70}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Polygon(
          points={{-78,-18},{-72,-18},{-52,-10},{-36,20},{74,24},{90,-18},{
              -94,-20},{-78,-18}},
          lineColor={28,108,200},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-4,34},{8,10}},
          lineColor={28,108,200},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-84,40},{92,32}},
          lineColor={28,108,200},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end UpperMachinery;
