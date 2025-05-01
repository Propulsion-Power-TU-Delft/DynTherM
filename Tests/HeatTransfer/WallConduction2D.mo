within DynTherM.Tests.HeatTransfer;
model WallConduction2D

  BoundaryConditions.OneDimensional.thermal1D heat_source_bottom_distributed(
    Nx=horizontal_wall.N,
    use_di_Q=false,
    use_in_Q=true)
    annotation (Placement(transformation(
        extent={{-20,12},{20,-12}},
        rotation=0,
        origin={0,-20})));
  BoundaryConditions.ZeroDimensional.thermal heat_source_left(
    use_Q=false,
    use_in_Q=true)
    annotation (Placement(transformation(extent={{-52,-6},{-34,6}})));
  Modelica.Blocks.Sources.Step input_left(
    height=-10,
    offset=10,
    startTime=30)
    annotation (Placement(transformation(extent={{-98,-14},{-82,2}})));
  Modelica.Blocks.Sources.Step input_bottom(height=10, startTime=30)
    annotation (Placement(transformation(extent={{-58,-58},{-40,-40}})));
  Components.TwoDimensional.WallConductionHorizontal2D horizontal_wall(
    x(displayUnit="mm") = 0.5,
    y(displayUnit="mm") = 0.01,
    z(displayUnit="mm") = 0.3,
    Tstart=298.15,
    N=10) annotation (Placement(transformation(extent={{-22,-22},{22,22}})));
  Components.TwoDimensional.WallConductionVertical2D vertical_wall(
    x(displayUnit="mm") = 0.01,
    y(displayUnit="mm") = 0.5,
    z(displayUnit="mm") = 0.3,
    Tstart=298.15,
    N=10) annotation (Placement(transformation(extent={{36,-4},{84,44}})));
  BoundaryConditions.ZeroDimensional.thermal heat_source_bottom(
    use_Q=false,
    use_in_Q=true)
    annotation (Placement(transformation(extent={{48,-26},{66,-14}})));
  BoundaryConditions.OneDimensional.thermal1D heat_source_left_distributed(
    Nx=vertical_wall.N,
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-20,12},{20,-12}},
        rotation=-90,
        origin={38,20})));
equation
  connect(input_left.y, heat_source_left.in_Q)
    annotation (Line(points={{-81.2,-6},{-53.8,-6}}, color={0,0,127}));
  connect(input_bottom.y, heat_source_bottom_distributed.in_Q) annotation (Line(
        points={{-39.1,-49},{-6.66667,-49},{-6.66667,-24.8}}, color={0,0,127}));
  connect(heat_source_bottom_distributed.thermal, horizontal_wall.South)
    annotation (Line(points={{0,-20},{0,-6.6}}, color={191,0,0}));
  connect(heat_source_left.thermal, horizontal_wall.West)
    annotation (Line(points={{-40,0},{-19.8,0}}, color={191,0,0}));
  connect(heat_source_left_distributed.thermal, vertical_wall.West)
    annotation (Line(points={{38,20},{52.8,20}}, color={191,0,0}));
  connect(heat_source_bottom.thermal, vertical_wall.South)
    annotation (Line(points={{60,-20},{60,-1.6}}, color={191,0,0}));
  connect(input_bottom.y, heat_source_bottom.in_Q) annotation (Line(points={{
          -39.1,-49},{32,-49},{32,-26},{46.2,-26}}, color={0,0,127}));
  connect(input_left.y, heat_source_left_distributed.in_Q) annotation (Line(
        points={{-81.2,-6},{-68,-6},{-68,26.6667},{33.2,26.6667}}, color={0,0,
          127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=60, __Dymola_Algorithm="Dassl"));
end WallConduction2D;
