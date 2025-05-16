within DynTherM.Tests.Distributed;
model WallHorizontal3D
  Components.ThreeDimensional.WallConductionHorizontal3D Wall(
    x=1,
    y(displayUnit="mm") = 0.01,
    z(displayUnit="mm") = 0.2,
    Nx=5,
    Nz=1) annotation (Placement(transformation(extent={{-20,40},{20,80}})));
  BoundaryConditions.TwoDimensional.thermal2D BCTop(
    Nx=Wall.Nx,
    Ny=Wall.Nz,
    T={{298.15},{298.15},{373.15},{298.15},{298.15}},
    use_di_Q=false,
    use_di_T=true)
    annotation (Placement(transformation(extent={{-22,72},{22,90}})));
  BoundaryConditions.TwoDimensional.thermal2D BCBottom(
    Nx=Wall.Nx,
    Ny=Wall.Nz,
    T={{298.15},{298.15},{298.15},{298.15},{298.15}},
    use_di_Q=false,
    use_di_T=true)
    annotation (Placement(transformation(extent={{-22,30},{22,48}})));
  Components.ThreeDimensional.WallConductionCV3D WallCV2(
    x=Wall.x/5,
    y=Wall.y,
    z=Wall.z,
    dm=0) annotation (Placement(transformation(extent={{-44,-12},{-18,14}})));
  Components.ThreeDimensional.WallConductionCV3D WallCV3(
    x=Wall.x/5,
    y=Wall.y,
    z=Wall.z,
    dm=0) annotation (Placement(transformation(extent={{-14,-12},{12,14}})));
  Components.ThreeDimensional.WallConductionCV3D WallCV4(
    x=Wall.x/5,
    y=Wall.y,
    z=Wall.z,
    dm=0) annotation (Placement(transformation(extent={{16,-12},{42,14}})));
  Components.ThreeDimensional.WallConductionCV3D WallCV5(
    x=Wall.x/5,
    y=Wall.y,
    z=Wall.z,
    dm=0) annotation (Placement(transformation(extent={{46,-12},{72,14}})));
  Components.ThreeDimensional.WallConductionCV3D WallCV1(
    x=Wall.x/5,
    y=Wall.y,
    z=Wall.z,
    dm=0) annotation (Placement(transformation(extent={{-74,-12},{-48,14}})));
  BoundaryConditions.ZeroDimensional.thermal BCTop1(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-66,16},{-58,22}})));
  BoundaryConditions.ZeroDimensional.thermal BCTop2(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-36,16},{-28,22}})));
  BoundaryConditions.ZeroDimensional.thermal BCTop3(
    T=373.15,
    use_Q=false,
    use_T=true) annotation (Placement(transformation(extent={{-6,16},{2,22}})));
  BoundaryConditions.ZeroDimensional.thermal BCTop4(
    T=298.15,
    use_Q=false,
    use_T=true) annotation (Placement(transformation(extent={{24,16},{32,22}})));
  BoundaryConditions.ZeroDimensional.thermal BCTop5(
    T=298.15,
    use_Q=false,
    use_T=true) annotation (Placement(transformation(extent={{54,16},{62,22}})));
  BoundaryConditions.ZeroDimensional.thermal BCBottom1(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-66,-22},{-58,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BCBottom2(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-36,-22},{-28,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BCBottom3(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-6,-22},{2,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BCBottom4(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{24,-22},{32,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BCBottom5(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{54,-22},{62,-16}})));
  Modelica.Blocks.Sources.Constant Q_int(k=0)
    annotation (Placement(transformation(extent={{-100,20},{-80,40}})));
  BoundaryConditions.ZeroDimensional.thermal BCWest(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-86,-4},{-78,2}})));
  BoundaryConditions.ZeroDimensional.thermal BCEast(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false) annotation (Placement(transformation(extent={{74,-4},{82,2}})));
  BoundaryConditions.ZeroDimensional.thermal BCFront1(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-72,-22},{-64,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BCFront2(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-42,-22},{-34,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BCFront3(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-12,-22},{-4,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BCFront4(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{18,-22},{26,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BCFront5(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{48,-22},{56,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BCRear1(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-54,16},{-46,22}})));
  BoundaryConditions.ZeroDimensional.thermal BCRear2(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-24,16},{-16,22}})));
  BoundaryConditions.ZeroDimensional.thermal BCRear3(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false) annotation (Placement(transformation(extent={{6,16},{14,22}})));
  BoundaryConditions.ZeroDimensional.thermal BCRear4(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{36,16},{44,22}})));
  BoundaryConditions.ZeroDimensional.thermal BCRear5(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{66,16},{74,22}})));
equation
  connect(BCTop.thermal, Wall.North)
    annotation (Line(points={{0,81},{0,65.4}}, color={191,0,0}));
  connect(Wall.South, BCBottom.thermal)
    annotation (Line(points={{0,54.6},{0,39}}, color={191,0,0}));
  connect(WallCV1.East, WallCV2.West)
    annotation (Line(points={{-53.2,-0.3},{-38.8,-0.3}}, color={191,0,0}));
  connect(WallCV2.East, WallCV3.West)
    annotation (Line(points={{-23.2,-0.3},{-8.8,-0.3}}, color={191,0,0}));
  connect(WallCV3.East, WallCV4.West)
    annotation (Line(points={{6.8,-0.3},{21.2,-0.3}}, color={191,0,0}));
  connect(WallCV4.East, WallCV5.West)
    annotation (Line(points={{36.8,-0.3},{51.2,-0.3}}, color={191,0,0}));
  connect(WallCV5.North, BCTop5.thermal) annotation (Line(points={{59,3.6},{
          59.3333,3.6},{59.3333,19}}, color={191,0,0}));
  connect(WallCV4.North, BCTop4.thermal) annotation (Line(points={{29,3.6},{
          29.3333,3.6},{29.3333,19}}, color={191,0,0}));
  connect(WallCV3.North, BCTop3.thermal) annotation (Line(points={{-1,3.6},{
          -0.666667,3.6},{-0.666667,19}}, color={191,0,0}));
  connect(WallCV2.North, BCTop2.thermal) annotation (Line(points={{-31,3.6},{
          -30.6667,3.6},{-30.6667,19}}, color={191,0,0}));
  connect(WallCV1.North, BCTop1.thermal) annotation (Line(points={{-61,3.6},{
          -60.6667,3.6},{-60.6667,19}}, color={191,0,0}));
  connect(BCBottom1.thermal, WallCV1.South) annotation (Line(points={{-60.6667,
          -19},{-61,-18},{-61,-4.2}}, color={191,0,0}));
  connect(BCBottom2.thermal, WallCV2.South) annotation (Line(points={{-30.6667,
          -19},{-31,-18},{-31,-4.2}}, color={191,0,0}));
  connect(BCBottom3.thermal, WallCV3.South) annotation (Line(points={{-0.666667,
          -19},{-1,-18},{-1,-4.2}}, color={191,0,0}));
  connect(BCBottom4.thermal, WallCV4.South) annotation (Line(points={{29.3333,
          -19},{29,-18},{29,-4.2}}, color={191,0,0}));
  connect(BCBottom5.thermal, WallCV5.South) annotation (Line(points={{59.3333,
          -19},{59,-18},{59,-4.2}}, color={191,0,0}));
  connect(Q_int.y, WallCV1.Q_int) annotation (Line(points={{-79,30},{-55.8,30},
          {-55.8,8.8}}, color={0,0,127}));
  connect(Q_int.y, WallCV2.Q_int) annotation (Line(points={{-79,30},{-25.8,30},
          {-25.8,8.8}}, color={0,0,127}));
  connect(Q_int.y, WallCV3.Q_int)
    annotation (Line(points={{-79,30},{4.2,30},{4.2,8.8}}, color={0,0,127}));
  connect(Q_int.y, WallCV4.Q_int)
    annotation (Line(points={{-79,30},{34.2,30},{34.2,8.8}}, color={0,0,127}));
  connect(Q_int.y, WallCV5.Q_int)
    annotation (Line(points={{-79,30},{64.2,30},{64.2,8.8}}, color={0,0,127}));
  connect(BCWest.thermal, WallCV1.West) annotation (Line(points={{-80.6667,-1},
          {-80.6667,-0.3},{-68.8,-0.3}}, color={191,0,0}));
  connect(WallCV5.East, BCEast.thermal) annotation (Line(points={{66.8,-0.3},{
          66.8,-1},{79.3333,-1}}, color={191,0,0}));
  connect(BCFront1.thermal, WallCV1.Front) annotation (Line(points={{-66.6667,
          -19},{-66.2,-19},{-66.2,-5.5}}, color={191,0,0}));
  connect(BCFront2.thermal, WallCV2.Front) annotation (Line(points={{-36.6667,
          -19},{-36.2,-18},{-36.2,-5.5}}, color={191,0,0}));
  connect(BCFront3.thermal, WallCV3.Front) annotation (Line(points={{-6.66667,
          -19},{-6.2,-18},{-6.2,-5.5}}, color={191,0,0}));
  connect(BCFront4.thermal, WallCV4.Front) annotation (Line(points={{23.3333,
          -19},{23.8,-18},{23.8,-5.5}}, color={191,0,0}));
  connect(BCFront5.thermal, WallCV5.Front) annotation (Line(points={{53.3333,
          -19},{53.8,-18},{53.8,-5.5}}, color={191,0,0}));
  connect(WallCV1.Rear, BCRear1.thermal) annotation (Line(points={{-55.8,4.9},{
          -48.6667,4.9},{-48.6667,19}}, color={191,0,0}));
  connect(WallCV2.Rear, BCRear2.thermal) annotation (Line(points={{-25.8,4.9},{
          -18.6667,4.9},{-18.6667,19}}, color={191,0,0}));
  connect(WallCV3.Rear, BCRear3.thermal) annotation (Line(points={{4.2,4.9},{
          11.3333,4.9},{11.3333,19}}, color={191,0,0}));
  connect(WallCV4.Rear, BCRear4.thermal) annotation (Line(points={{34.2,4.9},{
          41.3333,4.9},{41.3333,19}}, color={191,0,0}));
  connect(WallCV5.Rear, BCRear5.thermal) annotation (Line(points={{64.2,4.9},{
          71.3333,4.9},{71.3333,19}}, color={191,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=60, __Dymola_Algorithm="Dassl"));
end WallHorizontal3D;
