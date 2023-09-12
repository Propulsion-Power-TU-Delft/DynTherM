within ThermalManagement.Systems.Aircraft.Subsystems;
model FlightDeckWindshield
  "Model of flight deck windshield, made of glass-faced acrylic"
  // Reference: F. Zanghirella, et al. - A Numerical Model to Evaluate the thermal Behaviour of Active Transparent Facades, 2011.
  // Hp: air leakage from pressurized cabin to environment is neglected

  outer ThermalManagement.Components.Environment environment "Environmental properties";

  parameter Modelica.Units.SI.Length t_glass=0.005 "Thickness of the tempered glass layer";
  parameter Modelica.Units.SI.Length t_outer_acrylic=0.02 "Thickness of the external stretched acrylic layer";
  parameter Modelica.Units.SI.Length t_inner_acrylic=0.025 "Thickness of the internal stretched acrylic layer";
  parameter Modelica.Units.SI.Area A_windshield "Surface area of the windshield";
  parameter Modelica.Units.SI.Temperature Tstart "Tmperature of the window - starting value" annotation (Dialog(tab="Initialization"));

  Components.HeatTransfer.WindowRadiation glassRadiation(redeclare model Mat =
        ThermalManagement.Materials.TemperedGlass, A=A_windshield)
    annotation (Placement(transformation(extent={{-62,132},{2,68}})));
  Components.HeatTransfer.WallConduction glassConduction(
    redeclare model Mat = ThermalManagement.Materials.TemperedGlass,
    t=t_glass,
    A=A_windshield,
    Tstart=Tstart,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{16,30},{64,70}})));
  Components.HeatTransfer.WallConduction outerAcrylicConduction(
    redeclare model Mat = Materials.Opticor,
    t=t_outer_acrylic,
    A=A_windshield,
    Tstart=Tstart,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{16,-18},{64,22}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatAbsorbed annotation (
      Placement(transformation(extent={{30,-122},{50,-102}}),iconTransformation(
          extent={{10,-40},{30,-20}})));
  Components.HeatTransfer.WindowRadiation outerAcrylicRadiation(redeclare model
      Mat = Materials.Opticor, A=A_windshield)
    annotation (Placement(transformation(extent={{-72,82},{-8,18}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heatExt annotation (
      Placement(transformation(extent={{30,120},{50,140}}),
                                                          iconTransformation(
          extent={{10,40},{30,60}})));
  CustomInterfaces.IrradiancePort irradianceExt annotation (Placement(
        transformation(extent={{-36,126},{-26,136}}),
                                                    iconTransformation(extent={{
            -30,40},{-10,60}})));
  Components.Adaptors.irradianceToHeatFlow irradianceToHeatFlow(A=A_windshield)
    annotation (Placement(transformation(extent={{-92,-30},{-28,-94}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatTransmitted
    annotation (Placement(transformation(extent={{-70,-122},{-50,-102}}),
        iconTransformation(extent={{-30,-40},{-10,-20}})));
  Components.HeatTransfer.WindowRadiation innerAcrylicRadiation(redeclare model
      Mat = Materials.Opticor, A=A_windshield)
    annotation (Placement(transformation(extent={{-82,30},{-18,-34}})));
  Components.HeatTransfer.WallConduction innerAcrylicConduction(
    redeclare model Mat = Materials.Opticor,
    t=t_inner_acrylic,
    A=A_windshield,
    Tstart=Tstart,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{16,-70},{64,-30}})));
equation

  connect(glassRadiation.inlet_absorbed, glassConduction.inlet) annotation (
      Line(points={{-19.12,89.12},{-18,89.12},{-18,72},{40,72},{40,56.8}},
        color={191,0,0}));
  connect(glassRadiation.inlet_transmitted, outerAcrylicRadiation.outlet)
    annotation (Line(points={{-40.88,89.12},{-40.88,52},{-40,52},{-40,54.48}},
        color={191,0,0}));
  connect(glassRadiation.outlet, irradianceExt) annotation (Line(points={{-30,104.48},
          {-30,128.74},{-31,128.74},{-31,131}}, color={191,0,0}));
  connect(glassConduction.inlet, heatExt)
    annotation (Line(points={{40,56.8},{40,130}}, color={191,0,0}));
  connect(heatTransmitted, irradianceToHeatFlow.inlet)
    annotation (Line(points={{-60,-112},{-60,-81.2}},color={191,0,0}));
  connect(glassConduction.outlet, outerAcrylicConduction.inlet)
    annotation (Line(points={{40,43.2},{40,8.8}}, color={191,0,0}));
  connect(outerAcrylicRadiation.inlet_absorbed, outerAcrylicConduction.inlet)
    annotation (Line(points={{-29.12,39.12},{-29.12,20},{40,20},{40,8.8}},
        color={191,0,0}));
  connect(innerAcrylicRadiation.outlet, outerAcrylicRadiation.inlet_transmitted)
    annotation (Line(points={{-50,2.48},{-50,39.12},{-50.88,39.12}}, color={191,
          0,0}));
  connect(irradianceToHeatFlow.outlet, innerAcrylicRadiation.inlet_transmitted)
    annotation (Line(points={{-60,-42.8},{-60,-12.88},{-60.88,-12.88}}, color={191,
          0,0}));
  connect(outerAcrylicConduction.outlet, innerAcrylicConduction.inlet)
    annotation (Line(points={{40,-4.8},{40,-43.2}}, color={191,0,0}));
  connect(innerAcrylicConduction.outlet, heatAbsorbed)
    annotation (Line(points={{40,-56.8},{40,-112}}, color={191,0,0}));
  connect(innerAcrylicRadiation.inlet_absorbed, innerAcrylicConduction.inlet)
    annotation (Line(points={{-39.12,-12.88},{-39.12,-30},{40,-30},{40,-43.2}},
        color={191,0,0}));
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
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{100,140}})));
end FlightDeckWindshield;
