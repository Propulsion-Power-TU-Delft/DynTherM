within DynTherM.Systems.Helicopter.Subsystems;
model Window
  outer Components.Environment environment;
  parameter Modelica.Units.SI.Length L "Window length"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length W "Window width"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t=0.01 "Window thickness"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Angle csi
    "Tilt angle of the surface wrt horizontal"
    annotation (Dialog(group="Dimensions"));
  parameter Boolean use_r_eff=false "Use direct input for r_eff";
  parameter Real r_eff_di=0.02 "Reflection coefficient" annotation (Dialog(enable=use_r_eff));
  final parameter Modelica.Units.SI.Area A=L*W "Window area";

  replaceable model HTC_int =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    annotation (choicesAllMatching=true, Dialog(group="Heat Correlations"));

  replaceable model HTC_ext =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    annotation (choicesAllMatching=true, Dialog(group="Heat Correlations"));

  //Window Radiation
  Components.HeatTransfer.WindowRadiation windowRadiation(A=A, redeclare model
      Mat = Materials.AirbusEES.Window,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di)                                         annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-50,10})));
  //Window Conduction
  Components.HeatTransfer.WallConduction wallConduction(
    t=t,
    A=A,
    initOpt=environment.initOpt,
    redeclare model Mat = Materials.AirbusEES.Window)   annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-12,34})));
  //Window External Convection
  Components.HeatTransfer.ExternalConvection externalConvection(A=A, redeclare
      model HTC = HTC_ext)         annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-50,34})));
  //Window Internal Convection
  Components.HeatTransfer.InternalConvection internalConvection(A=A, redeclare
      model HTC = HTC_int)         annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={14,34})));
  //Port
  CustomInterfaces.IrradiancePort irradiancePort
    annotation (Placement(transformation(extent={{-100,0},{-80,20}}),
        iconTransformation(extent={{-100,0},{-80,20}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a thermalPortInner
    annotation (Placement(transformation(extent={{80,26},{100,46}}),
        iconTransformation(extent={{80,26},{100,46}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a thermalPortToSeats
    annotation (Placement(transformation(extent={{80,-2},{100,18}}),
        iconTransformation(extent={{80,-40},{100,-20}})));
equation
  connect(externalConvection.inlet,wallConduction.outlet) annotation (
      Line(points={{-46.6,34},{-15.4,34}},                   color={191,0,0}));
  connect(internalConvection.outlet,wallConduction.inlet)
    annotation (Line(points={{10.6,34},{-8.6,34}},  color={191,0,0}));
  connect(irradiancePort, windowRadiation.outlet)
    annotation (Line(points={{-90,10},{-51.4,10}}, color={191,0,0}));
  connect(externalConvection.inlet, wallConduction.outlet) annotation (Line(
        points={{-46.6,34},{-15.4,34}},                   color={191,0,0}));
  connect(windowRadiation.inlet_absorbed, wallConduction.outlet) annotation (
      Line(points={{-46.6,12.6},{-16,12.6},{-16,34},{-15.4,34}}, color={191,0,0}));
  connect(thermalPortToSeats,thermalPortToSeats)
    annotation (Line(points={{90,8},{90,8}},   color={191,0,0}));
  connect(windowRadiation.inlet_transmitted, thermalPortToSeats)
    annotation (Line(points={{-46.6,7.4},{90,7.4},{90,8}}, color={191,0,0}));
  connect(internalConvection.inlet, thermalPortInner)
    annotation (Line(points={{17.4,34},{90,34},{90,36}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{100,-10},{-100,10}},
          fillColor={85,255,255},
          fillPattern=FillPattern.Backward,
          lineColor={0,0,0},
          origin={-36,0},
          rotation=-90)}),                                       Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Window;
