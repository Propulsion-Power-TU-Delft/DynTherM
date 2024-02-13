﻿within DynTherM.Systems.Aircraft.Subsystems;
model CabinFloor
  "Simplified model of cabin floor separating cabin and cargo bay"

  outer DynTherM.Components.Environment environment "Environmental properties";
  parameter Modelica.Units.SI.Temperature Tstart=300 "Temperature start value"
    annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Area A "Surface of the cabin floor";

  final parameter Modelica.Units.SI.Length t_tot=0.0254 "Floor thcikness";
  final parameter Modelica.Units.SI.Length t_fl_f=0.000762
    "Floor facing thickness";
  final parameter Modelica.Units.SI.Length t_fl_c=t_tot - 2*t_fl_f
    "Floor core thickness";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b ext annotation (Placement(
        transformation(extent={{-10,60},{10,80}}), iconTransformation(extent={{-10,
            60},{10,80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a int annotation (Placement(
        transformation(extent={{-10,-80},{10,-60}}), iconTransformation(extent={
            {-10,-80},{10,-60}})));
  Components.HeatTransfer.WallConduction floor_ext(
    redeclare model Mat = Materials.CarbonPhenolic,
    t=t_fl_f,
    A=A,
    Tstart=Tstart,
    initOpt=environment.initOpt) "Material: carbon phenolic"
    annotation (Placement(transformation(extent={{-28,46},{28,14}})));
  Components.HeatTransfer.WallConduction floor_core(
    redeclare model Mat = Materials.FibrelamAramid6100,
    t=t_fl_c,
    A=A,
    Tstart=Tstart - 1,
    initOpt=environment.initOpt)
    "Material: Fibrelam® 6100 Aramid phenolic honeycomb HRH-10-1/8-9.0"
    annotation (Placement(transformation(extent={{-28,16},{28,-16}})));
  Components.HeatTransfer.WallConduction floor_int(
    redeclare model Mat = Materials.CarbonPhenolic,
    t=t_fl_f,
    A=A,
    Tstart=Tstart - 1.5,
    initOpt=environment.initOpt) "Material: carbon phenolic"
    annotation (Placement(transformation(extent={{-28,-14},{28,-46}})));
equation
  connect(int, floor_int.inlet) annotation (Line(points={{0,-70},{3.9968e-15,-70},
          {3.9968e-15,-35.44}}, color={191,0,0}));
  connect(floor_ext.outlet, ext)
    annotation (Line(points={{0,35.44},{0,70}}, color={191,0,0}));
  connect(floor_int.outlet, floor_core.inlet)
    annotation (Line(points={{0,-24.56},{0,-5.44}},  color={191,0,0}));
  connect(floor_core.outlet, floor_ext.inlet)
    annotation (Line(points={{0,5.44},{0,24.56}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                      Rectangle(
          extent={{-100,60},{100,20}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
                      Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward),
                      Rectangle(
          extent={{-100,-20},{100,-60}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Text(
          extent={{-54,32},{56,-34}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward,
          textString="FLOOR")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><b>References:</b></p>
<p>[1]&nbsp;L.&nbsp;Krakers.&nbsp;Parametric&nbsp;fuselage&nbsp;design:&nbsp;integration&nbsp;of&nbsp;mechanics,&nbsp;acoustic&nbsp;and&nbsp;thermal&nbsp;insulation,&nbsp;2009.</p>
<p>[2]&nbsp;SAE&nbsp;AIR&nbsp;1168/3,&nbsp;section&nbsp;7.2</p>
<p>[3]&nbsp;Gillfab&nbsp;4117&nbsp;product&nbsp;data&nbsp;sheet</p>
</html>"));
end CabinFloor;
