within DynTherM.Tests.HeatTransfer;
model ConductionWallSteady
  "Validation test case of steady-state conduction (wall). Reference: es.8, pag.92 fisica tecnica"
  BoundaryConditions.ZeroDimensional.thermal cold_side(
    T=273.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-20,-98},{10,-80}})));
  Components.HeatTransfer.WallConduction wall_A(
    redeclare model Mat = DynTherM.Materials.Aluminium,
    t=0.1,
    A=0.4,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{-20,46},{20,8}})));
  Components.HeatTransfer.WallConduction wall_D(
    redeclare model Mat = DynTherM.Materials.Aluminium,
    t=0.1,
    A=0.4,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{-20,-6},{20,-44}})));
  BoundaryConditions.ZeroDimensional.thermal hot_side(
    T=373.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-20,80},{10,98}})));
  Components.HeatTransfer.WallConduction wall_B(
    redeclare model Mat = DynTherM.Materials.Aluminium,
    t=0.1,
    A=0.18,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{-60,20},{-20,-18}})));
  Components.HeatTransfer.WallConduction wall_C(
    redeclare model Mat = DynTherM.Materials.Aluminium,
    t=0.1,
    A=0.22,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{20,20},{60,-18}})));
  Modelica.Thermal.HeatTransfer.Components.Convection convection_1 annotation (
      Placement(transformation(
        extent={{-11,-12},{11,12}},
        rotation=90,
        origin={0,61})));
  Modelica.Thermal.HeatTransfer.Components.Convection convection_2 annotation (
      Placement(transformation(
        extent={{11,-12},{-11,12}},
        rotation=90,
        origin={0,-59})));
  Modelica.Blocks.Sources.Constant const(k=4)
    annotation (Placement(transformation(extent={{-48,52},{-30,70}})));
  Modelica.Blocks.Sources.Constant const1(k=1.6)
    annotation (Placement(transformation(extent={{-50,-68},{-32,-50}})));
equation
  connect(wall_D.outlet, wall_B.inlet) annotation (Line(points={{
          4.44089e-16,-18.54},{-40,-18.54},{-40,-5.46}},
                                     color={191,0,0}));
  connect(wall_B.outlet, wall_A.inlet) annotation (Line(points={{-40,7.46},
          {-40,20.54},{3.33067e-16,20.54}},
                                       color={191,0,0}));
  connect(wall_C.outlet, wall_A.inlet) annotation (Line(points={{40,7.46},{
          40,20.54},{3.33067e-16,20.54}},
                                color={191,0,0}));
  connect(wall_A.outlet, convection_1.solid)
    annotation (Line(points={{3.33067e-16,33.46},{3.33067e-16,42},{0,42},{0,
          50}},                                 color={191,0,0}));
  connect(convection_1.fluid, hot_side.thermal)
    annotation (Line(points={{0,72},{0,89}}, color={191,0,0}));
  connect(wall_D.inlet, convection_2.solid)
    annotation (Line(points={{4.44089e-16,-31.46},{4.44089e-16,-40},{0,-40},
          {0,-48}},                               color={191,0,0}));
  connect(convection_2.fluid,cold_side. thermal)
    annotation (Line(points={{0,-70},{0,-89}}, color={191,0,0}));
  connect(const.y, convection_1.Gc)
    annotation (Line(points={{-29.1,61},{-12,61}}, color={0,0,127}));
  connect(const1.y, convection_2.Gc) annotation (Line(points={{-31.1,-59},{-21.55,
          -59},{-21.55,-59},{-12,-59}}, color={0,0,127}));
  connect(wall_D.outlet, wall_C.inlet) annotation (Line(points={{
          4.44089e-16,-18.54},{40,-18.54},{40,-5.46}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=100, Interval=0.1));
end ConductionWallSteady;
