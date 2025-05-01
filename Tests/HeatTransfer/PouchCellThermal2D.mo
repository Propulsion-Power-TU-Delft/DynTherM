within DynTherM.Tests.HeatTransfer;
model PouchCellThermal2D
  // Unit test
  XogenyTest.AssertFinal check_T1_60s(actual=cell.cv[1].T_vol, expected=299.234, eps=0.1);
  XogenyTest.AssertFinal check_T2_60s(actual=cell.cv[2].T_vol, expected=298.503, eps=0.1);
  XogenyTest.AssertFinal check_T3_60s(actual=cell.cv[3].T_vol, expected=298.443, eps=0.1);
  XogenyTest.AssertValueAt check_T1_30s(actual=cell.cv[1].T_vol, at=30, expected=298.438, eps=0.1);
  XogenyTest.AssertValueAt check_T2_30s(actual=cell.cv[2].T_vol, at=30, expected=298.438, eps=0.1);
  XogenyTest.AssertValueAt check_T3_30s(actual=cell.cv[3].T_vol, at=30, expected=298.438, eps=0.1);

  Components.TwoDimensional.PouchCellThermal2D cell(
    W(displayUnit="mm") = 0.35,
    t(displayUnit="mm") = 0.01,
    H(displayUnit="mm") = 0.1,
    dm=0,
    Tstart=298.15,
    initOpt=DynTherM.Choices.InitOpt.fixedState,
    N=3) annotation (Placement(transformation(extent={{-42,-42},{44,42}})));
  BoundaryConditions.OneDimensional.thermal1D heat_source_left(
    Nx=3,
    Q=10*ones(3),
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-20,12},{20,-12}},
        rotation=-90,
        origin={-60,0})));
  BoundaryConditions.ZeroDimensional.thermal heat_source_bottom(use_Q=false,
      use_in_Q=true)
    annotation (Placement(transformation(extent={{-12,-66},{6,-54}})));
  Modelica.Blocks.Sources.Step input_left(
    height=-10,
    offset=10,
    startTime=30)
    annotation (Placement(transformation(extent={{-100,-2},{-82,16}})));
  Modelica.Blocks.Sources.Step input_bottom(height=10, startTime=30)
    annotation (Placement(transformation(extent={{-60,-76},{-40,-56}})));
equation
  connect(heat_source_left.thermal, cell.Left)
    annotation (Line(points={{-60,0},{-34.69,0}}, color={191,0,0}));
  connect(heat_source_bottom.thermal, cell.Bottom) annotation (Line(points={{0,
          -60},{0.14,-60},{0.14,-26.04}}, color={191,0,0}));
  connect(input_left.y, heat_source_left.in_Q) annotation (Line(points={{-81.1,
          7},{-78,6.66667},{-64.8,6.66667}}, color={0,0,127}));
  connect(input_bottom.y, heat_source_bottom.in_Q)
    annotation (Line(points={{-39,-66},{-13.8,-66}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=60, __Dymola_Algorithm="Dassl"));
end PouchCellThermal2D;
