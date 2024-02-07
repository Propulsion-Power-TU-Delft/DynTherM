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
    annotation (Placement(transformation(extent={{-62,100},{2,36}})));
  Components.HeatTransfer.WallConduction outerLayerConduction(
    redeclare model Mat = Materials.Opticor,
      t=t_outer,
      A=A_window,
    Tstart=Tstart*ones(1, 1),
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{16,-2},{64,38}})));
  Components.HeatTransfer.WallConduction innerLayerConduction(
    redeclare model Mat = Materials.Opticor,
      t=t_inner,
      A=A_window,
    Tstart=Tstart*ones(1, 1),
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{16,-86},{64,-46}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatAbsorbed annotation (
      Placement(transformation(extent={{80,-102},{100,-82}}),iconTransformation(
          extent={{10,-40},{30,-20}})));
  Components.HeatTransfer.EnclosedAirSpace enclosedAirSpace(
    initOpt=environment.initOpt,
    w=t_air_gap,
    h=H_window,
    l=L_window,
    delta=1.553343034275)
    annotation (Placement(transformation(extent={{16,-42},{64,-6}})));
  Components.HeatTransfer.WindowRadiation innerLayerRadiation(
    redeclare model Mat = Materials.Opticor, A=A_window)
    annotation (Placement(transformation(extent={{-72,38},{-8,-26}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heatExt annotation (
      Placement(transformation(extent={{30,78},{50,98}}), iconTransformation(
          extent={{10,40},{30,60}})));
  CustomInterfaces.IrradiancePort irradianceExt annotation (Placement(
        transformation(extent={{-34,86},{-26,94}}), iconTransformation(extent={{
            -30,40},{-10,60}})));
  CustomInterfaces.Adaptors.irradianceToHeatFlow irradianceToHeatFlow(A=
        A_window)
    annotation (Placement(transformation(extent={{-82,-16},{-18,-80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatTransmitted
    annotation (Placement(transformation(extent={{-60,-102},{-40,-82}}),
        iconTransformation(extent={{-30,-40},{-10,-20}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier2(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{-30,48},{-8,26}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier1(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{28,62},{52,40}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier3(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{-40,-18},{-18,-40}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier4(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{28,-36},{52,-58}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier5(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{28,-12},{52,10}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier6(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{28,-96},{52,-74}})));
equation
  enclosedAirSpace.P = P_air;
  enclosedAirSpace.X = X_air;

  connect(outerLayerRadiation.inlet_transmitted, innerLayerRadiation.outlet)
    annotation (Line(points={{-40.88,57.12},{-40.88,10},{-40,10},{-40,10.48}},
        color={191,0,0}));
  connect(outerLayerRadiation.outlet, irradianceExt) annotation (Line(points={{-30,
          72.48},{-30,90}},                         color={191,0,0}));
  connect(heatTransmitted, irradianceToHeatFlow.inlet)
    annotation (Line(points={{-50,-92},{-50,-67.2}}, color={191,0,0}));
  connect(irradianceToHeatFlow.outlet, innerLayerRadiation.inlet_transmitted)
    annotation (Line(points={{-50,-28.8},{-50,-6.84},{-50.88,-6.84},{-50.88,
          -4.88}}, color={191,0,0}));
  connect(outerLayerRadiation.inlet_absorbed, heatFlowMultiplier2.single)
    annotation (Line(points={{-19.12,57.12},{-19.12,42},{-19,42},{-19,43.6}},
        color={191,0,0}));
  connect(heatFlowMultiplier2.distributed, outerLayerConduction.inlet)
    annotation (Line(points={{-19,30.4},{40,30.4},{40,23.6}}, color={191,0,0}));
  connect(heatExt, heatFlowMultiplier1.single)
    annotation (Line(points={{40,88},{40,57.6}}, color={191,0,0}));
  connect(heatFlowMultiplier1.distributed, outerLayerConduction.inlet)
    annotation (Line(points={{40,44.4},{40,23.6}}, color={191,0,0}));
  connect(innerLayerRadiation.inlet_absorbed, heatFlowMultiplier3.single)
    annotation (Line(points={{-29.12,-4.88},{-29,-4.88},{-29,-22.4}}, color={
          191,0,0}));
  connect(heatFlowMultiplier3.distributed, innerLayerConduction.inlet)
    annotation (Line(points={{-29,-35.6},{-29,-60.4},{40,-60.4}}, color={191,0,
          0}));
  connect(heatFlowMultiplier6.single, heatAbsorbed)
    annotation (Line(points={{40,-91.6},{40,-92},{90,-92}}, color={191,0,0}));
  connect(innerLayerConduction.outlet, heatFlowMultiplier6.distributed)
    annotation (Line(points={{40,-71.6},{40,-78.4}}, color={191,0,0}));
  connect(heatFlowMultiplier4.distributed, innerLayerConduction.inlet)
    annotation (Line(points={{40,-53.6},{40,-60.4}}, color={191,0,0}));
  connect(enclosedAirSpace.outlet, heatFlowMultiplier4.single)
    annotation (Line(points={{40,-30.12},{40,-40.4}}, color={191,0,0}));
  connect(heatFlowMultiplier5.single, enclosedAirSpace.inlet)
    annotation (Line(points={{40,-7.6},{40,-17.88}}, color={191,0,0}));
  connect(outerLayerConduction.outlet, heatFlowMultiplier5.distributed)
    annotation (Line(points={{40,12.4},{40,5.6}}, color={191,0,0}));
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
