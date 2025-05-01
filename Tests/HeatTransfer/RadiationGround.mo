within DynTherM.Tests.HeatTransfer;
model RadiationGround
  "Validation test case for thermal radiation. Reference: example 8, pag. 298, ASHRAE Handbook – Fundamentals, chapter 14, 2013."
  Components.HeatTransfer.SolarRadiation thermalRadiation1(csi=0)
    annotation (Placement(transformation(extent={{-16,98},{16,66}})));
  BoundaryConditions.ZeroDimensional.thermal int1(
    T=293.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-16,32},{8,48}})));
  inner Components.Environment environment(
    altitude_di(displayUnit="km") = 11000,
    V_inf_di=0,
    ISA_plus=0,
    psi=0,
    Day=202,
    Hour=14,
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{66,66},{100,100}})));
  Components.HeatTransfer.WallRadiation wallRadiation1(A=1, csi=0)
    annotation (Placement(transformation(extent={{-12,70},{12,46}})));
  Components.HeatTransfer.SolarRadiation thermalRadiation2(csi=
        0.78539816339745)
    annotation (Placement(transformation(extent={{-56,98},{-24,66}})));
  BoundaryConditions.ZeroDimensional.thermal int2(
    T=293.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-56,32},{-32,48}})));
  Components.HeatTransfer.WallRadiation wallRadiation2(A=1, csi=
        0.78539816339745)
    annotation (Placement(transformation(extent={{-52,70},{-28,46}})));
  Components.HeatTransfer.SolarRadiation thermalRadiation8(csi=
        5.4977871437821)
             annotation (Placement(transformation(extent={{24,98},{56,66}})));
  BoundaryConditions.ZeroDimensional.thermal int8(
    T=293.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{24,32},{48,48}})));
  Components.HeatTransfer.WallRadiation wallRadiation8(A=1, csi=
        5.4977871437821)
    annotation (Placement(transformation(extent={{28,70},{52,46}})));
  Components.HeatTransfer.SolarRadiation thermalRadiation5(csi=
        3.1415926535898)
    annotation (Placement(transformation(extent={{-16,-98},{16,-66}})));
  BoundaryConditions.ZeroDimensional.thermal int5(
    T=293.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-16,-32},{8,-48}})));
  Components.HeatTransfer.WallRadiation wallRadiation5(A=1, csi=
        3.1415926535898)
    annotation (Placement(transformation(extent={{-12,-70},{12,-46}})));
  Components.HeatTransfer.SolarRadiation thermalRadiation6(csi=
        3.9269908169872)
    annotation (Placement(transformation(extent={{24,-98},{56,-66}})));
  BoundaryConditions.ZeroDimensional.thermal int6(
    T=293.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{24,-32},{48,-48}})));
  Components.HeatTransfer.WallRadiation wallRadiation6(A=1, csi=
        3.9269908169872)
    annotation (Placement(transformation(extent={{28,-70},{52,-46}})));
  Components.HeatTransfer.SolarRadiation thermalRadiation4(csi=
        2.3561944901923)
    annotation (Placement(transformation(extent={{-56,-98},{-24,-66}})));
  BoundaryConditions.ZeroDimensional.thermal int4(
    T=293.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-56,-32},{-32,-48}})));
  Components.HeatTransfer.WallRadiation wallRadiation4(A=1, csi=
        2.3561944901923)
    annotation (Placement(transformation(extent={{-52,-70},{-28,-46}})));
  Components.HeatTransfer.SolarRadiation thermalRadiation3(csi=
        1.5707963267949) annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=-90,
        origin={-82,0})));
  BoundaryConditions.ZeroDimensional.thermal int3(
    T=293.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-24,8},{-48,-8}})));
  Components.HeatTransfer.WallRadiation wallRadiation3(A=1, csi=
        1.5707963267949) annotation (Placement(transformation(
        extent={{-12,12},{12,-12}},
        rotation=90,
        origin={-56,0})));
  Components.HeatTransfer.SolarRadiation thermalRadiation7(csi=
        4.7123889803847)
             annotation (Placement(transformation(extent={{-16,16},{16,-16}},
        rotation=-90,
        origin={82,0})));

  BoundaryConditions.ZeroDimensional.thermal int7(
    T=293.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{24,8},{48,-8}})));
  Components.HeatTransfer.WallRadiation wallRadiation7(A=1, csi=
        4.7123889803847)
    annotation (Placement(transformation(extent={{-12,-12},{12,12}},
        rotation=90,
        origin={56,0})));
equation
  connect(thermalRadiation1.inlet, wallRadiation1.outlet)
    annotation (Line(points={{0,72.08},{0,59.68}}, color={191,0,0}));
  connect(wallRadiation1.inlet, int1.thermal)
    annotation (Line(points={{0,53.92},{0,40}}, color={191,0,0}));
  connect(thermalRadiation2.inlet, wallRadiation2.outlet)
    annotation (Line(points={{-40,72.08},{-40,59.68}}, color={191,0,0}));
  connect(wallRadiation2.inlet, int2.thermal)
    annotation (Line(points={{-40,53.92},{-40,40}}, color={191,0,0}));
  connect(thermalRadiation8.inlet, wallRadiation8.outlet)
    annotation (Line(points={{40,72.08},{40,59.68}}, color={191,0,0}));
  connect(wallRadiation8.inlet, int8.thermal)
    annotation (Line(points={{40,53.92},{40,40}}, color={191,0,0}));
  connect(thermalRadiation5.inlet, wallRadiation5.outlet)
    annotation (Line(points={{0,-72.08},{0,-59.68}}, color={191,0,0}));
  connect(wallRadiation5.inlet, int5.thermal)
    annotation (Line(points={{0,-53.92},{0,-40}}, color={191,0,0}));
  connect(thermalRadiation6.inlet, wallRadiation6.outlet)
    annotation (Line(points={{40,-72.08},{40,-59.68}}, color={191,0,0}));
  connect(wallRadiation6.inlet, int6.thermal)
    annotation (Line(points={{40,-53.92},{40,-40}}, color={191,0,0}));
  connect(thermalRadiation4.inlet, wallRadiation4.outlet)
    annotation (Line(points={{-40,-72.08},{-40,-59.68}}, color={191,0,0}));
  connect(wallRadiation4.inlet, int4.thermal)
    annotation (Line(points={{-40,-53.92},{-40,-40}}, color={191,0,0}));
  connect(thermalRadiation3.inlet, wallRadiation3.outlet)
    annotation (Line(points={{-72.08,0},{-57.68,0}}, color={191,0,0}));
  connect(wallRadiation3.inlet, int3.thermal)
    annotation (Line(points={{-51.92,0},{-40,0}}, color={191,0,0}));
  connect(thermalRadiation7.inlet, wallRadiation7.outlet)
    annotation (Line(points={{72.08,0},{57.68,0}}, color={191,0,0}));
  connect(wallRadiation7.inlet, int7.thermal)
    annotation (Line(points={{51.92,0},{40,0}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RadiationGround;
