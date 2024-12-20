within DynTherM.Systems.Aircraft.Subsystems;
model FuselageSandwichStructure
  "Model of sandwich shell fuselage (cabin and cockpit sections)"

  // Initialization
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart "Temperature start value - external skin" annotation (Dialog(tab="Initialization"));

  // Geometry
  parameter Real coeff "Fraction of fuselage with active heat transfer";
  parameter Length L_fuselage
    "Length of the fuselage cylindrical section";
  input Length R_ext "External radius of the fuselage" annotation (Dialog(enable=true));
  parameter Length t_tot=0.1
    "Overall fuselage thickness (half for lower section)";
  parameter Length L_window=0 "Window length";
  parameter Length H_window=0 "Window height";
  parameter Integer Nw_side=0 "Number of windows per fuselage side";

  final parameter Length t_f=0.00136 "Skin facing thickness";
  final parameter Length t_c=0.012 "Skin core thickness";
  final parameter Length t_in_f=0.0005
    "Interior panel facing thickness";
  final parameter Length t_in_c=0.004
    "Interior panel core thickness";
  final parameter Length t_ins=t_tot - (2*t_f + t_c) - (2*
      t_in_f + t_in_c) "Insulation blankets thickness";

  Components.HeatTransfer.TubeConduction skin_ext(
    redeclare model Mat = Materials.CarbonPhenolic,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext,
    R_int=R_ext - t_f,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=Tstart,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-26,74},{26,44}})));
  Components.HeatTransfer.TubeConduction skin_core(
    redeclare model Mat = Materials.Hexcel,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - t_f,
    R_int=R_ext - t_f - t_c,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=Tstart - 0.5,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-26,56},{26,26}})));
  Components.HeatTransfer.TubeConduction skin_int(
    redeclare model Mat = Materials.CarbonPhenolic,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - t_f - t_c,
    R_int=R_ext - 2*t_f - t_c,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=Tstart - 1,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-26,38},{26,8}})));
  Components.HeatTransfer.TubeConduction insulationBlankets(
    redeclare model Mat = Materials.GlassFibre,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - 2*t_f - t_c,
    R_int=R_ext - 2*t_f - t_c - t_ins,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=Tstart - 1.5,
    initOpt=initOpt)
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
    Tstart=Tstart - 2,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-26,-8},{26,-38}})));
  Components.HeatTransfer.TubeConduction interiorPanel_core(
    redeclare model Mat = Materials.FibrelamAramid1100,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - 2*t_f - t_c - t_ins - t_in_f,
    R_int=R_ext - 2*t_f - t_c - t_ins - t_in_f - t_in_c,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=Tstart - 2.5,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-26,-26},{26,-56}})));
  Components.HeatTransfer.TubeConduction interiorPanel_int(
    redeclare model Mat = Materials.GlassPhenolic,
    coeff=coeff,
    L=L_fuselage,
    R_ext=R_ext - 2*t_f - t_c - t_ins - t_in_f - t_in_c,
    R_int=R_ext - 2*t_f - t_c - t_ins - 2*t_in_f - t_in_c,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=Tstart - 3,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-26,-44},{26,-74}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b ext
    annotation (Placement(transformation(extent={{-10,70},{10,90}}),
        iconTransformation(extent={{-10,60},{10,80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a int
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}}),
        iconTransformation(extent={{-10,-80},{10,-60}})));
equation
  connect(skin_ext.inlet,skin_core. outlet)
    annotation (Line(points={{4.44089e-16,53.9},{4.44089e-16,46.1}},
                                                   color={191,0,0}));
  connect(skin_core.inlet,skin_int. outlet)
    annotation (Line(points={{4.44089e-16,35.9},{4.44089e-16,28.1}},
                                                   color={191,0,0}));
  connect(skin_int.inlet,insulationBlankets. outlet)
    annotation (Line(points={{4.44089e-16,17.9},{4.44089e-16,6.1}},
                                                   color={191,0,0}));
  connect(interiorPanel_ext.inlet,interiorPanel_core. outlet)
    annotation (Line(points={{4.44089e-16,-28.1},{4.44089e-16,-35.9}},
                                                   color={191,0,0}));
  connect(interiorPanel_core.inlet,interiorPanel_int. outlet)
    annotation (Line(points={{4.44089e-16,-46.1},{4.44089e-16,-53.9}},
                                                     color={191,0,0}));
  connect(ext, skin_ext.outlet)
    annotation (Line(points={{0,80},{0,64.1}}, color={191,0,0}));
  connect(int, interiorPanel_int.inlet)
    annotation (Line(points={{0,-80},{0,-64.1}}, color={191,0,0}));
  connect(insulationBlankets.inlet, interiorPanel_ext.outlet)
    annotation (Line(points={{0,-4.1},{0,-17.9}}, color={191,0,0}));
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
<p><b>Assumption:</b>&nbsp;fin&nbsp;effect&nbsp;of&nbsp;structural&nbsp;members&nbsp;attached&nbsp;to&nbsp;the&nbsp;skin&nbsp;and&nbsp;protruding&nbsp;into&nbsp;the&nbsp;cabin&nbsp;is&nbsp;neglected.</p>
<p><br><b>References:</b></p>
<p>[1]&nbsp;L.&nbsp;Krakers.&nbsp;Parametric&nbsp;fuselage&nbsp;design:&nbsp;integration&nbsp;of&nbsp;mechanics,&nbsp;acoustic&nbsp;and&nbsp;thermal&nbsp;insulation,&nbsp;2009.</p>
<p>[2]&nbsp;SAE&nbsp;AIR&nbsp;1168/3,&nbsp;section&nbsp;7.2</p>
</html>"));
end FuselageSandwichStructure;
