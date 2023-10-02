within DynTherM.Systems.Aircraft.Subsystems;
model CabinWindow
  "Model of cabin window, featuring double glazing and air cavity"

  outer DynTherM.Components.Environment environment "Environmental properties";

  parameter Modelica.Units.SI.Length t_outer=0.01 "Thickness of the external layer";
  parameter Modelica.Units.SI.Length t_inner=0.005 "Thickness of the internal layer";
  parameter Modelica.Units.SI.Length t_air_gap=0.007 "Thickness of the air gap between the external and the internal layers";
  parameter Modelica.Units.SI.Length H_window "Height of the window";
  parameter Modelica.Units.SI.Length L_window "Length of the window";
  parameter Modelica.Units.SI.Temperature Tstart "Tmperature of the window - starting value" annotation (Dialog(tab="Initialization"));
  final parameter Modelica.Units.SI.Area A_window=L_window*H_window "Surface area of the window";

  input Modelica.Units.SI.Pressure P_air "Average pressure inside the cabin window air cavity";
  input Modelica.Units.SI.MassFraction X_air[2] "Composition of the cabin window air cavity";

  Components.HeatTransfer.WindowRadiation outerLayerRadiation(
    redeclare model Mat = Materials.Opticor, A=A_window)
    annotation (Placement(transformation(extent={{-62,92},{2,28}})));
  Components.HeatTransfer.WallConduction outerLayerConduction(
    redeclare model Mat = Materials.Opticor,
      t=t_outer,
      A=A_window,
      Tstart=Tstart,
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{16,-16},{64,24}})));
  Components.HeatTransfer.WallConduction innerLayerConduction(
    redeclare model Mat = Materials.Opticor,
      t=t_inner,
      A=A_window,
      Tstart=Tstart,
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{16,-80},{64,-40}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatAbsorbed annotation (
      Placement(transformation(extent={{30,-102},{50,-82}}), iconTransformation(
          extent={{10,-40},{30,-20}})));
  Components.HeatTransfer.EnclosedAirSpace enclosedAirSpace(
    initOpt=environment.initOpt,
    w=t_air_gap,
    h=H_window,
    l=L_window,
    delta=1.553343034275)
    annotation (Placement(transformation(extent={{16,-46},{64,-10}})));
  Components.HeatTransfer.WindowRadiation innerLayerRadiation(
    redeclare model Mat = Materials.Opticor, A=A_window)
    annotation (Placement(transformation(extent={{-72,38},{-8,-26}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heatExt annotation (
      Placement(transformation(extent={{30,78},{50,98}}), iconTransformation(
          extent={{10,40},{30,60}})));
  CustomInterfaces.IrradiancePort irradianceExt annotation (Placement(
        transformation(extent={{-36,84},{-26,94}}), iconTransformation(extent={{
            -30,40},{-10,60}})));
  Components.Adaptors.irradianceToHeatFlow irradianceToHeatFlow(A=A_window)
    annotation (Placement(transformation(extent={{-82,-16},{-18,-80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatTransmitted
    annotation (Placement(transformation(extent={{-60,-102},{-40,-82}}),
        iconTransformation(extent={{-30,-40},{-10,-20}})));
equation
  enclosedAirSpace.P = P_air;
  enclosedAirSpace.X = X_air;

  connect(outerLayerRadiation.inlet_absorbed, outerLayerConduction.inlet)
    annotation (Line(points={{-19.12,49.12},{-18,49.12},{-18,24},{40,24},{40,
          10.8}},                                                         color=
         {191,0,0}));
  connect(heatAbsorbed, innerLayerConduction.outlet)
    annotation (Line(points={{40,-92},{40,-66.8}}, color={191,0,0}));
  connect(enclosedAirSpace.inlet, outerLayerConduction.outlet)
    annotation (Line(points={{40,-21.88},{40,-2.8}},  color={191,0,0}));
  connect(innerLayerConduction.inlet, enclosedAirSpace.outlet)
    annotation (Line(points={{40,-53.2},{40,-34.12}}, color={191,0,0}));
  connect(outerLayerRadiation.inlet_transmitted, innerLayerRadiation.outlet)
    annotation (Line(points={{-40.88,49.12},{-40.88,10},{-40,10},{-40,10.48}},
        color={191,0,0}));
  connect(innerLayerRadiation.inlet_absorbed, innerLayerConduction.inlet)
    annotation (Line(points={{-29.12,-4.88},{-29.12,-53.2},{40,-53.2}},  color={
          191,0,0}));
  connect(outerLayerRadiation.outlet, irradianceExt) annotation (Line(points={{-30,
          64.48},{-30,86.74},{-31,86.74},{-31,89}}, color={191,0,0}));
  connect(outerLayerConduction.inlet, heatExt)
    annotation (Line(points={{40,10.8},{40,88}}, color={191,0,0}));
  connect(heatTransmitted, irradianceToHeatFlow.inlet)
    annotation (Line(points={{-50,-92},{-50,-67.2}}, color={191,0,0}));
  connect(irradianceToHeatFlow.outlet, innerLayerRadiation.inlet_transmitted)
    annotation (Line(points={{-50,-28.8},{-50,-6.84},{-50.88,-6.84},{-50.88,
          -4.88}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,80}}),                                         graphics={
        Rectangle(
          extent={{-100,40},{100,20}},
          fillColor={85,255,255},
          fillPattern=FillPattern.Backward,
          lineColor={0,0,0}),
        Rectangle(
          extent={{-100,0},{100,-20}},
          fillColor={85,255,255},
          fillPattern=FillPattern.Backward,
          lineColor={0,0,0}),
                      Rectangle(
          extent={{-100,20},{100,0}},
          lineColor={0,0,0}),
        Line(points={{-80,-40},{-80,60}}, color={238,46,47}),
        Line(points={{-60,-40},{-60,60}}, color={238,46,47}),
        Line(points={{-80,60},{-86,50}},  color={238,46,47}),
        Line(points={{-80,60},{-74,50}},  color={238,46,47}),
        Line(points={{-60,60},{-54,50}},  color={238,46,47}),
        Line(points={{-60,60},{-66,50}},  color={238,46,47}),
        Line(points={{60,-40},{60,60}},   color={238,46,47}),
        Line(points={{80,-40},{80,60}},   color={238,46,47}),
        Line(points={{74,-30},{80,-40}}, color={238,46,47}),
        Line(points={{54,-30},{60,-40}}, color={238,46,47}),
        Line(points={{86,-30},{80,-40}}, color={238,46,47}),
        Line(points={{66,-30},{60,-40}}, color={238,46,47}),
        Line(points={{40,10},{-40,10}},   color={0,127,255}),
        Line(points={{-3,5},{3,-5}},      color={0,127,255},
          origin={35,7},
          rotation=-90),
        Line(points={{3,5},{-3,-5}},      color={0,127,255},
          origin={35,13},
          rotation=-90)}),                                       Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
    Documentation(info="<html>
<p>Hp: air leakage from pressurized cabin to environment is neglected.</p>
<p>Reference:</p>
<p>[1] F.&nbsp;Zanghirella,&nbsp;et&nbsp;al. &quot;A&nbsp;Numerical&nbsp;Model&nbsp;to&nbsp;Evaluate&nbsp;the&nbsp;thermal&nbsp;Behaviour&nbsp;of&nbsp;Active&nbsp;Transparent&nbsp;Facades&quot;,&nbsp;2011.</p>
</html>"));
end CabinWindow;
