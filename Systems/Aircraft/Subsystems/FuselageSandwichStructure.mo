within DynTherM.Systems.Aircraft.Subsystems;
model FuselageSandwichStructure
  "Model of sandwich shell fuselage (cabin and cockpit sections)"

  outer DynTherM.Components.Environment environment "Environmental properties";
  parameter Real coeff "Fraction of fuselage with active heat transfer";
  parameter Modelica.Units.SI.Length L_fuselage
    "Length of the fuselage cylindrical section";
  parameter Modelica.Units.SI.Length R_ext "External radius of the fuselage";
  parameter Modelica.Units.SI.Length t_tot=0.1
    "Overall fuselage thickness (half for lower section)";
  parameter Modelica.Units.SI.Length L_window=0 "Window length";
  parameter Modelica.Units.SI.Length H_window=0 "Window height";
  parameter Integer Nw_side=0 "Number of windows per fuselage side";
  parameter Modelica.Units.SI.Temperature Tstart
    "Temperature start value - external skin"
    annotation (Dialog(tab="Initialization"));

  final parameter Modelica.Units.SI.Length t_f=0.00136 "Skin facing thickness";
  final parameter Modelica.Units.SI.Length t_c=0.012 "Skin core thickness";
  final parameter Modelica.Units.SI.Length t_in_f=0.0005
    "Interior panel facing thickness";
  final parameter Modelica.Units.SI.Length t_in_c=0.004
    "Interior panel core thickness";
  final parameter Modelica.Units.SI.Length t_ins=t_tot - (2*t_f + t_c) - (2*
      t_in_f + t_in_c) "Insulation blankets thickness";
  constant Real pi=Modelica.Constants.pi;
  Components.HeatTransfer.TubeConduction skin_ext(
    redeclare model Mat = Materials.CarbonPhenolic,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext,
    R_int=R_ext - t_f,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=Tstart*ones(1, 1),
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-26,70},{26,40}})));
  Components.HeatTransfer.TubeConduction skin_core(
    redeclare model Mat = Materials.Hexcel,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - t_f,
    R_int=R_ext - t_f - t_c,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=(Tstart - 0.5)*ones(1, 1),
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-26,52},{26,22}})));
  Components.HeatTransfer.TubeConduction skin_int(
    redeclare model Mat = Materials.CarbonPhenolic,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - t_f - t_c,
    R_int=R_ext - 2*t_f - t_c,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=(Tstart - 1)*ones(1, 1),
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-26,34},{26,4}})));
  Components.HeatTransfer.TubeConduction insulationBlankets(
    redeclare model Mat = Materials.GlassFibre,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - 2*t_f - t_c,
    R_int=R_ext - 2*t_f - t_c - t_ins,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=(Tstart - 1.5)*ones(1, 1),
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-26,16},{26,-14}})));
  Components.HeatTransfer.TubeConduction interiorPanel_ext(
    redeclare model Mat = Materials.GlassPhenolic,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - 2*t_f - t_c - t_ins,
    R_int=R_ext - 2*t_f - t_c - t_ins - t_in_f,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=(Tstart - 2)*ones(1, 1),
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-26,-4},{26,-34}})));
  Components.HeatTransfer.TubeConduction interiorPanel_core(
    redeclare model Mat = Materials.FibrelamAramid1100,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - 2*t_f - t_c - t_ins - t_in_f,
    R_int=R_ext - 2*t_f - t_c - t_ins - t_in_f - t_in_c,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=(Tstart - 2.5)*ones(1, 1),
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-26,-22},{26,-52}})));
  Components.HeatTransfer.TubeConduction interiorPanel_int(
    redeclare model Mat = Materials.GlassPhenolic,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - 2*t_f - t_c - t_ins - t_in_f - t_in_c,
    R_int=R_ext - 2*t_f - t_c - t_ins - 2*t_in_f - t_in_c,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=(Tstart - 3)*ones(1, 1),
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-26,-40},{26,-70}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b ext
    annotation (Placement(transformation(extent={{-10,80},{10,100}}),
        iconTransformation(extent={{-10,60},{10,80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a int
    annotation (Placement(transformation(extent={{-10,-100},{10,-80}}),
        iconTransformation(extent={{-10,-80},{10,-60}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{-10,-82},{10,-62}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier1(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{-10,82},{10,62}})));
equation
  connect(skin_ext.inlet,skin_core. outlet)
    annotation (Line(points={{4.44089e-16,50.8},{4.44089e-16,41.2}},
                                                   color={191,0,0}));
  connect(skin_core.inlet,skin_int. outlet)
    annotation (Line(points={{4.44089e-16,32.8},{4.44089e-16,23.2}},
                                                   color={191,0,0}));
  connect(skin_int.inlet,insulationBlankets. outlet)
    annotation (Line(points={{4.44089e-16,14.8},{4.44089e-16,5.2}},
                                                   color={191,0,0}));
  connect(interiorPanel_ext.inlet,interiorPanel_core. outlet)
    annotation (Line(points={{4.44089e-16,-23.2},{4.44089e-16,-32.8}},
                                                   color={191,0,0}));
  connect(interiorPanel_core.inlet,interiorPanel_int. outlet)
    annotation (Line(points={{4.44089e-16,-41.2},{4.44089e-16,-50.8}},
                                                     color={191,0,0}));
  connect(insulationBlankets.inlet, interiorPanel_ext.outlet)
    annotation (Line(points={{0,-3.2},{0,-14.8}}, color={191,0,0}));
  connect(interiorPanel_int.inlet, heatFlowMultiplier.distributed)
    annotation (Line(points={{0,-59.2},{0,-66}}, color={191,0,0}));
  connect(heatFlowMultiplier.single, int)
    annotation (Line(points={{0,-78},{0,-90}}, color={191,0,0}));
  connect(heatFlowMultiplier1.distributed, skin_ext.outlet)
    annotation (Line(points={{0,66},{0,59.2}}, color={191,0,0}));
  connect(ext, heatFlowMultiplier1.single)
    annotation (Line(points={{0,90},{0,78}}, color={191,0,0}));
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
          extent={{-80,38},{76,-40}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward,
          textString="FUSELAGE")}),                              Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Hp:&nbsp;fin&nbsp;effect&nbsp;of&nbsp;structural&nbsp;members&nbsp;attached&nbsp;to&nbsp;the&nbsp;skin&nbsp;and&nbsp;protruding&nbsp;into&nbsp;the&nbsp;cabin&nbsp;is&nbsp;neglected&nbsp;in&nbsp;first&nbsp;approximation.</p>
<p>References:</p>
<p>[1]&nbsp;L.&nbsp;Krakers.&nbsp;Parametric&nbsp;fuselage&nbsp;design:&nbsp;integration&nbsp;of&nbsp;mechanics,&nbsp;acoustic&nbsp;and&nbsp;thermal&nbsp;insulation,&nbsp;2009.</p>
<p>[2]&nbsp;SAE&nbsp;AIR&nbsp;1168/3,&nbsp;section&nbsp;7.2</p>
</html>"));
end FuselageSandwichStructure;
