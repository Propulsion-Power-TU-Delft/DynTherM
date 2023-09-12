within ThermalManagement.Systems.Helicopter.Subsystems;
model SandwichPanel
  "Sandwich panel model of NH90 - Aluminium and FibrelamAramid6100"
  // Reference (composite material):
  // M. Van Tooren, L. Krakers. Multi-disciplinary Design of Aircraft Fuselage Structures, 2007.
  // Hp: same materials and composite structure of cabin floor
  outer ThermalManagement.Components.Environment environment "Environmental properties";
  parameter Modelica.Units.SI.Temperature T_start=300
    "Temperature start value of inner panel"
    annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Length L "Panel length";
  parameter Modelica.Units.SI.Length W "Panel width";
  parameter Modelica.Units.SI.Area A_window=0 "Window area";
  parameter Integer Nw_side=0 "Number of windows per fuselage side";
  parameter Modelica.Units.SI.Length t_total(displayUnit="mm") = 0.0096
    "Total thickness";
  final parameter Modelica.Units.SI.Length t_o(displayUnit="mm") = t_total*1/12
    "Outer thickness";
  final parameter Modelica.Units.SI.Length t_c(displayUnit="mm") = t_total*5/6
    "Core thickness";
  final parameter Modelica.Units.SI.Length t_i(displayUnit="mm") = t_total*1/12
    "Inner thickness" annotation (enable=Symmetric);
  Modelica.Units.SI.ThermalConductivity lambda
    "Overall thermal conductivity of panel";
  final parameter Modelica.Units.SI.Temperature T_o(fixed=false);

  // Materials
  replaceable model Fuselage_int =
      ThermalManagement.Materials.AirbusEES.Fuselage                              constrainedby
    ThermalManagement.Materials.Properties
    "Internal fuselage material" annotation (choicesAllMatching=true, Dialog(group="Materials"));
  replaceable model Fuselage_core =
      ThermalManagement.Materials.AirbusEES.Fuselage                               constrainedby
    ThermalManagement.Materials.Properties
    "Core fuselage material" annotation (choicesAllMatching=true, Dialog(group="Materials"));
  replaceable model Fuselage_ext =
      ThermalManagement.Materials.AirbusEES.Fuselage                              constrainedby
    ThermalManagement.Materials.Properties
    "External fuselage material" annotation (choicesAllMatching=true, Dialog(group="Materials"));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b ext
    annotation (Placement(transformation(extent={{-10,60},{10,80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a int
    annotation (Placement(transformation(extent={{-10,-80},{10,-60}})));
  Components.HeatTransfer.WallConduction wall_ext(
    t=t_o,
    A=W*L,
    initOpt=environment.initOpt,
    A_window=A_window,
    Nw_side=Nw_side,
    Tstart=T_o,
    redeclare model Mat = Fuselage_ext)
    annotation (Placement(transformation(extent={{-28,46},{28,14}})));
  Components.HeatTransfer.WallConduction wall_core(
    t=t_c,
    A=W*L,
    initOpt=environment.initOpt,
    A_window=A_window,
    Nw_side=Nw_side,
    Tstart=(wall_ext.Tstart + wall_int.Tstart)/2,
    redeclare model Mat = Fuselage_core)
    annotation (Placement(transformation(extent={{-28,16},{28,-16}})));
  Components.HeatTransfer.WallConduction wall_int(
    t=t_i,
    A=W*L,
    initOpt=environment.initOpt,
    A_window=A_window,
    Nw_side=Nw_side,
    Tstart=T_start,
    redeclare model Mat = Fuselage_int)
    annotation (Placement(transformation(extent={{-28,-14},{28,-46}})));
initial equation
   T_o = environment.T_amb;
equation
  t_total/lambda = t_i/wall_int.Mat.lambda + t_c/wall_core.Mat.lambda+ t_o/wall_ext.Mat.lambda;
  connect(int, wall_int.inlet)
    annotation (Line(points={{0,-70},{0,-35.44}}, color={191,0,0}));

  connect(wall_int.outlet, wall_core.inlet)
    annotation (Line(points={{0,-24.56},{0,-5.44}}, color={191,0,0}));
  connect(wall_core.outlet, wall_ext.inlet)
    annotation (Line(points={{0,5.44},{0,24.56}}, color={191,0,0}));
  connect(wall_ext.outlet, ext)
    annotation (Line(points={{0,35.44},{0,70}}, color={191,0,0}));
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
          extent={{-84,52},{84,-50}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward,
          textString="FUSELAGE")}),                              Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SandwichPanel;
