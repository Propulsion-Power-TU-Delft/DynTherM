within DynTherM.Systems.Aircraft.Subsystems;
model FlightDeckWindshield
  "Model of flight deck windshield, made of glass-faced acrylic"

  outer DynTherM.Components.Environment environment "Environmental properties";

  parameter Modelica.Units.SI.Length t_glass=0.005 "Thickness of the tempered glass layer";
  parameter Modelica.Units.SI.Length t_outer_acrylic=0.02 "Thickness of the external stretched acrylic layer";
  parameter Modelica.Units.SI.Length t_inner_acrylic=0.025 "Thickness of the internal stretched acrylic layer";
  parameter Modelica.Units.SI.Area A_windshield "Surface area of the windshield";
  parameter Modelica.Units.SI.Temperature Tstart "Tmperature of the window - starting value" annotation (Dialog(tab="Initialization"));

  Components.HeatTransfer.WindowRadiation glassRadiation(redeclare model Mat =
        DynTherM.Materials.TemperedGlass,          A=A_windshield)
    annotation (Placement(transformation(extent={{-62,144},{2,80}})));
  Components.HeatTransfer.WallConduction glassConduction(
    redeclare model Mat = DynTherM.Materials.TemperedGlass,
    t=t_glass,
    A=A_windshield,
    Tstart=Tstart*ones(1, 1),
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{16,38},{64,78}})));
  Components.HeatTransfer.WallConduction outerAcrylicConduction(
    redeclare model Mat = Materials.Opticor,
    t=t_outer_acrylic,
    A=A_windshield,
    Tstart=Tstart*ones(1, 1),
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{16,-18},{64,22}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatAbsorbed annotation (
      Placement(transformation(extent={{30,-122},{50,-102}}),iconTransformation(
          extent={{10,-40},{30,-20}})));
  Components.HeatTransfer.WindowRadiation outerAcrylicRadiation(redeclare model
      Mat = Materials.Opticor, A=A_windshield)
    annotation (Placement(transformation(extent={{-72,88},{-8,24}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heatExt annotation (
      Placement(transformation(extent={{30,120},{50,140}}),
                                                          iconTransformation(
          extent={{10,40},{30,60}})));
  CustomInterfaces.IrradiancePort irradianceExt annotation (Placement(
        transformation(extent={{-34,128},{-26,136}}),
                                                    iconTransformation(extent={{
            -30,40},{-10,60}})));
  CustomInterfaces.Adaptors.irradianceToHeatFlow irradianceToHeatFlow(A=
        A_windshield)
    annotation (Placement(transformation(extent={{-92,-30},{-28,-94}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatTransmitted
    annotation (Placement(transformation(extent={{-70,-122},{-50,-102}}),
        iconTransformation(extent={{-30,-40},{-10,-20}})));
  Components.HeatTransfer.WindowRadiation innerAcrylicRadiation(redeclare model
      Mat = Materials.Opticor, A=A_windshield)
    annotation (Placement(transformation(extent={{-82,34},{-18,-30}})));
  Components.HeatTransfer.WallConduction innerAcrylicConduction(
    redeclare model Mat = Materials.Opticor,
    t=t_inner_acrylic,
    A=A_windshield,
    Tstart=Tstart*ones(1, 1),
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{16,-72},{64,-32}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier2(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{-30,92},{-8,70}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier1(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{28,112},{52,90}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier3(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{-40,36},{-18,14}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier4(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{-50,-18},{-28,-40}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier5(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{28,-94},{52,-72}})));
equation

  connect(glassRadiation.inlet_transmitted, outerAcrylicRadiation.outlet)
    annotation (Line(points={{-40.88,101.12},{-40.88,52},{-40,52},{-40,60.48}},
        color={191,0,0}));
  connect(glassRadiation.outlet, irradianceExt) annotation (Line(points={{-30,
          116.48},{-30,132}},                   color={191,0,0}));
  connect(heatTransmitted, irradianceToHeatFlow.inlet)
    annotation (Line(points={{-60,-112},{-60,-81.2}},color={191,0,0}));
  connect(glassConduction.outlet, outerAcrylicConduction.inlet)
    annotation (Line(points={{40,52.4},{40,7.6}}, color={191,0,0}));
  connect(innerAcrylicRadiation.outlet, outerAcrylicRadiation.inlet_transmitted)
    annotation (Line(points={{-50,6.48},{-50,45.12},{-50.88,45.12}}, color={191,
          0,0}));
  connect(irradianceToHeatFlow.outlet, innerAcrylicRadiation.inlet_transmitted)
    annotation (Line(points={{-60,-42.8},{-60,-8.88},{-60.88,-8.88}},   color={191,
          0,0}));
  connect(outerAcrylicConduction.outlet, innerAcrylicConduction.inlet)
    annotation (Line(points={{40,-3.6},{40,-46.4}}, color={191,0,0}));
  connect(glassRadiation.inlet_absorbed, heatFlowMultiplier2.single)
    annotation (Line(points={{-19.12,101.12},{-19,101.12},{-19,87.6}}, color={
          191,0,0}));
  connect(heatFlowMultiplier2.distributed, glassConduction.inlet) annotation (
      Line(points={{-19,74.4},{40,74.4},{40,63.6}}, color={191,0,0}));
  connect(heatFlowMultiplier1.distributed, glassConduction.inlet)
    annotation (Line(points={{40,94.4},{40,63.6}}, color={191,0,0}));
  connect(heatExt, heatFlowMultiplier1.single)
    annotation (Line(points={{40,130},{40,107.6}}, color={191,0,0}));
  connect(outerAcrylicRadiation.inlet_absorbed, heatFlowMultiplier3.single)
    annotation (Line(points={{-29.12,45.12},{-29,45.12},{-29,31.6}}, color={191,
          0,0}));
  connect(heatFlowMultiplier3.distributed, outerAcrylicConduction.inlet)
    annotation (Line(points={{-29,18.4},{40,18.4},{40,7.6}}, color={191,0,0}));
  connect(innerAcrylicRadiation.inlet_absorbed, heatFlowMultiplier4.single)
    annotation (Line(points={{-39.12,-8.88},{-39,-8.88},{-39,-22.4}}, color={
          191,0,0}));
  connect(heatFlowMultiplier4.distributed, innerAcrylicConduction.inlet)
    annotation (Line(points={{-39,-35.6},{40,-35.6},{40,-46.4}}, color={191,0,0}));
  connect(heatFlowMultiplier5.distributed, innerAcrylicConduction.outlet)
    annotation (Line(points={{40,-76.4},{40,-57.6}}, color={191,0,0}));
  connect(heatFlowMultiplier5.single, heatAbsorbed)
    annotation (Line(points={{40,-89.6},{40,-112}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},
            {100,140}}),                                        graphics={
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
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{100,140}})),
    Documentation(info="<html>
<p>Hp: air leakage from pressurized cabin to environment is neglected.</p>
<p>Reference:</p>
<p>[1] F.&nbsp;Zanghirella,&nbsp;et&nbsp;al.&nbsp;&quot;A&nbsp;Numerical&nbsp;Model&nbsp;to&nbsp;Evaluate&nbsp;the&nbsp;thermal&nbsp;Behaviour&nbsp;of&nbsp;Active&nbsp;Transparent&nbsp;Facades&quot;,&nbsp;2011.</p>
</html>"));
end FlightDeckWindshield;
