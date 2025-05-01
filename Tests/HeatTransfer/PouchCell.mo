within DynTherM.Tests.HeatTransfer;
model PouchCell
  // Unit test
  XogenyTest.AssertFinal check_T1_60s(actual=cell.thermal.cv[1].T_vol, expected=299.234, eps=0.1);
  XogenyTest.AssertFinal check_T2_60s(actual=cell.thermal.cv[2].T_vol, expected=298.503, eps=0.1);
  XogenyTest.AssertFinal check_T3_60s(actual=cell.thermal.cv[3].T_vol, expected=298.443, eps=0.1);
  XogenyTest.AssertValueAt check_T1_30s(actual=cell.thermal.cv[1].T_vol, at=30, expected=298.438, eps=0.1);
  XogenyTest.AssertValueAt check_T2_30s(actual=cell.thermal.cv[2].T_vol, at=30, expected=298.438, eps=0.1);
  XogenyTest.AssertValueAt check_T3_30s(actual=cell.thermal.cv[3].T_vol, at=30, expected=298.438, eps=0.1);

  Components.Electrical.PouchCell cell(
    W(displayUnit="mm") = 0.35,
    t(displayUnit="mm") = 0.01,
    H(displayUnit="mm") = 0.1,
    C_nom=230400,
    SoC_start=0.1,
    Tstart=298.15,
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    N=3) annotation (Placement(transformation(extent={{-44,-30},{44,28}})));
  BoundaryConditions.OneDimensional.thermal1D heat_source_left(
    Nx=3,
    Q=10*ones(3),
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-20,12},{20,-12}},
        rotation=-90,
        origin={-60,0})));
  BoundaryConditions.ZeroDimensional.thermal heat_source_bottom(
    use_Q=false,
    use_in_Q=true)
    annotation (Placement(transformation(extent={{-12,-66},{6,-54}})));
  Modelica.Blocks.Sources.Step input_left(
    height=-10,
    offset=10,
    startTime=30)
    annotation (Placement(transformation(extent={{-100,-2},{-82,16}})));
  Modelica.Blocks.Sources.Step input_bottom(height=10, startTime=30)
    annotation (Placement(transformation(extent={{-60,-76},{-40,-56}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{52,-16},{68,0}})));
equation
  connect(input_left.y, heat_source_left.in_Q) annotation (Line(points={{-81.1,
          7},{-78,6.66667},{-64.8,6.66667}}, color={0,0,127}));
  connect(input_bottom.y, heat_source_bottom.in_Q)
    annotation (Line(points={{-39,-66},{-13.8,-66}}, color={0,0,127}));
  connect(heat_source_left.thermal, cell.Left) annotation (Line(points={{-60,0},
          {-25.9111,0},{-25.9111,-1}}, color={191,0,0}));
  connect(heat_source_bottom.thermal, cell.Bottom)
    annotation (Line(points={{0,-60},{0,-24.2}}, color={191,0,0}));
  connect(cell.p, ground.p) annotation (Line(points={{34.7111,8.18333},{60,
          8.18333},{60,0}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=60, __Dymola_Algorithm="Dassl"));
end PouchCell;
