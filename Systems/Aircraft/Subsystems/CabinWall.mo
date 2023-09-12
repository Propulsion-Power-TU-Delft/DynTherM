within ThermalManagement.Systems.Aircraft.Subsystems;
model CabinWall "Simplified model of cabin wall separating cabin and cockpit"
  // Hp: same structure and materials as cabin floor

  outer ThermalManagement.Components.Environment environment "Environmental properties";
  parameter Modelica.Units.SI.Temperature Tstart=300 "Temperature start value"
    annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Area A "Surface of the cabin wall";
  parameter Modelica.Units.SI.Area A_window=0 "Window area";
  parameter Integer Nw_side=0 "Number of windows per fuselage side";

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
    A_window=A_window,
    Nw_side=Nw_side,
    Tstart=Tstart,
    initOpt=environment.initOpt) "Material: carbon phenolic"
    annotation (Placement(transformation(extent={{-28,46},{28,14}})));
  Components.HeatTransfer.WallConduction floor_core(
    redeclare model Mat = Materials.FibrelamAramid6100,
    t=t_fl_c,
    A=A,
    A_window=A_window,
    Nw_side=Nw_side,
    Tstart=Tstart - 1,
    initOpt=environment.initOpt)
    "Material: Fibrelam® 6100 Aramid phenolic honeycomb HRH-10-1/8-9.0"
    annotation (Placement(transformation(extent={{-28,16},{28,-16}})));
  Components.HeatTransfer.WallConduction floor_int(
    redeclare model Mat = Materials.CarbonPhenolic,
    t=t_fl_f,
    A=A,
    A_window=A_window,
    Nw_side=Nw_side,
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
          textString="WALL")}),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CabinWall;
