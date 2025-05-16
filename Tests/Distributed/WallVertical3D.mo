within DynTherM.Tests.Distributed;
model WallVertical3D
  Components.ThreeDimensional.WallConductionVertical3D Wall(
    x(displayUnit="mm") = 0.01,
    y(displayUnit="m") = 1,
    z(displayUnit="mm") = 0.2,
    Ny=5,
    Nz=1) annotation (Placement(transformation(extent={{-20,40},{20,80}})));
  BoundaryConditions.TwoDimensional.thermal2D BCWest(
    Nx=Wall.Ny,
    Ny=Wall.Nz,
    T={{298.15},{298.15},{373.15},{298.15},{298.15}},
    use_di_Q=false,
    use_di_T=true) annotation (Placement(transformation(
        extent={{-23.5,-9.5},{23.5,9.5}},
        rotation=-90,
        origin={-40.5,60.5})));
  BoundaryConditions.TwoDimensional.thermal2D BCEast(
    Nx=Wall.Ny,
    Ny=Wall.Nz,
    T={{298.15},{298.15},{298.15},{298.15},{298.15}},
    use_di_Q=false,
    use_di_T=true) annotation (Placement(transformation(
        extent={{-22.5,-9.5},{22.5,9.5}},
        rotation=90,
        origin={40.5,60.5})));
  Components.ThreeDimensional.WallConductionCV3D WallCV2(
    x=Wall.x,
    y=Wall.y/5,
    z=Wall.z,
    dm=0) annotation (Placement(transformation(extent={{-16,-22},{10,4}})));
  Components.ThreeDimensional.WallConductionCV3D WallCV3(
    x=Wall.x,
    y=Wall.y/5,
    z=Wall.z,
    dm=0) annotation (Placement(transformation(extent={{-16,-42},{10,-16}})));
  Components.ThreeDimensional.WallConductionCV3D WallCV4(
    x=Wall.x,
    y=Wall.y/5,
    z=Wall.z,
    dm=0) annotation (Placement(transformation(extent={{-16,-62},{10,-36}})));
  Components.ThreeDimensional.WallConductionCV3D WallCV5(
    x=Wall.x,
    y=Wall.y/5,
    z=Wall.z,
    dm=0) annotation (Placement(transformation(extent={{-16,-82},{10,-56}})));
  Components.ThreeDimensional.WallConductionCV3D WallCV1(
    x=Wall.x,
    y=Wall.y/5,
    z=Wall.z,
    dm=0) annotation (Placement(transformation(extent={{-16,-2},{10,24}})));
  Modelica.Blocks.Sources.Constant Q_int(k=0)
    annotation (Placement(transformation(extent={{-100,20},{-80,40}})));
  BoundaryConditions.ZeroDimensional.thermal BCFront1(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-26,-4},{-18,2}})));
  BoundaryConditions.ZeroDimensional.thermal BCFront2(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-26,-24},{-18,-18}})));
  BoundaryConditions.ZeroDimensional.thermal BCFront3(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-26,-44},{-18,-38}})));
  BoundaryConditions.ZeroDimensional.thermal BCFront4(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-26,-64},{-18,-58}})));
  BoundaryConditions.ZeroDimensional.thermal BCFront5(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-26,-84},{-18,-78}})));
  BoundaryConditions.ZeroDimensional.thermal BCRear1(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{14,18},{22,24}})));
  BoundaryConditions.ZeroDimensional.thermal BCRear2(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false) annotation (Placement(transformation(extent={{14,-2},{22,4}})));
  BoundaryConditions.ZeroDimensional.thermal BCRear3(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{14,-22},{22,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BCRear4(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{14,-42},{22,-36}})));
  BoundaryConditions.ZeroDimensional.thermal BCRear5(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{14,-62},{22,-56}})));
  BoundaryConditions.ZeroDimensional.thermal BCWest1(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-26,6},{-18,12}})));
  BoundaryConditions.ZeroDimensional.thermal BCWest2(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-26,-14},{-18,-8}})));
  BoundaryConditions.ZeroDimensional.thermal BCWest3(
    T=373.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-26,-34},{-18,-28}})));
  BoundaryConditions.ZeroDimensional.thermal BCWest4(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-26,-54},{-18,-48}})));
  BoundaryConditions.ZeroDimensional.thermal BCWest5(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-26,-74},{-18,-68}})));
  BoundaryConditions.ZeroDimensional.thermal BCEast1(
    T=298.15,
    use_Q=false,
    use_T=true) annotation (Placement(transformation(extent={{14,6},{22,12}})));
  BoundaryConditions.ZeroDimensional.thermal BCEast2(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{14,-14},{22,-8}})));
  BoundaryConditions.ZeroDimensional.thermal BCEast3(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{14,-34},{22,-28}})));
  BoundaryConditions.ZeroDimensional.thermal BCEast4(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{14,-54},{22,-48}})));
  BoundaryConditions.ZeroDimensional.thermal BCEast5(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{14,-74},{22,-68}})));
  BoundaryConditions.ZeroDimensional.thermal BCTop(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false) annotation (Placement(transformation(extent={{-8,22},{0,28}})));
  BoundaryConditions.ZeroDimensional.thermal BCBottom(
    T=298.15,
    Q=0,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-8,-90},{0,-84}})));
equation
  connect(BCWest.thermal, Wall.West) annotation (Line(points={{-40.5,60.5},{-38,
          60},{-5.2,60}}, color={191,0,0}));
  connect(Wall.East, BCEast.thermal)
    annotation (Line(points={{5.2,60},{6,60.5},{40.5,60.5}}, color={191,0,0}));
  connect(WallCV2.North, WallCV1.South)
    annotation (Line(points={{-3,-6.4},{-3,5.8}}, color={191,0,0}));
  connect(WallCV3.North, WallCV2.South)
    annotation (Line(points={{-3,-26.4},{-3,-14.2}}, color={191,0,0}));
  connect(WallCV4.North, WallCV3.South)
    annotation (Line(points={{-3,-46.4},{-3,-34.2}}, color={191,0,0}));
  connect(WallCV5.North, WallCV4.South)
    annotation (Line(points={{-3,-66.4},{-3,-54.2}}, color={191,0,0}));
  connect(Q_int.y, WallCV1.Q_int)
    annotation (Line(points={{-79,30},{2.2,30},{2.2,18.8}}, color={0,0,127}));
  connect(Q_int.y, WallCV2.Q_int)
    annotation (Line(points={{-79,30},{2.2,30},{2.2,-1.2}}, color={0,0,127}));
  connect(Q_int.y, WallCV3.Q_int)
    annotation (Line(points={{-79,30},{2.2,30},{2.2,-21.2}}, color={0,0,127}));
  connect(Q_int.y, WallCV4.Q_int)
    annotation (Line(points={{-79,30},{2.2,30},{2.2,-41.2}}, color={0,0,127}));
  connect(Q_int.y, WallCV5.Q_int)
    annotation (Line(points={{-79,30},{2.2,30},{2.2,-61.2}}, color={0,0,127}));
  connect(BCFront1.thermal, WallCV1.Front) annotation (Line(points={{-20.6667,
          -1},{-8.2,-1},{-8.2,4.5}},
                                 color={191,0,0}));
  connect(BCFront2.thermal, WallCV2.Front) annotation (Line(points={{-20.6667,
          -21},{-8.2,-21},{-8.2,-15.5}},
                                    color={191,0,0}));
  connect(BCFront3.thermal, WallCV3.Front) annotation (Line(points={{-20.6667,
          -41},{-8.2,-41},{-8.2,-35.5}},
                                    color={191,0,0}));
  connect(BCFront4.thermal, WallCV4.Front) annotation (Line(points={{-20.6667,
          -61},{-8.2,-61},{-8.2,-55.5}},
                                    color={191,0,0}));
  connect(BCFront5.thermal, WallCV5.Front) annotation (Line(points={{-20.6667,
          -81},{-8.2,-81},{-8.2,-75.5}},
                                    color={191,0,0}));
  connect(WallCV5.Rear, BCRear5.thermal) annotation (Line(points={{2.2,-65.1},{
          19.3333,-65.1},{19.3333,-59}},
                                 color={191,0,0}));
  connect(WallCV4.Rear, BCRear4.thermal) annotation (Line(points={{2.2,-45.1},{
          19.3333,-45.1},{19.3333,-39}},
                                 color={191,0,0}));
  connect(WallCV3.Rear, BCRear3.thermal) annotation (Line(points={{2.2,-25.1},{
          19.3333,-25.1},{19.3333,-19}},
                                 color={191,0,0}));
  connect(WallCV2.Rear, BCRear2.thermal) annotation (Line(points={{2.2,-5.1},{
          19.3333,-5.1},{19.3333,1}},
                              color={191,0,0}));
  connect(WallCV1.Rear, BCRear1.thermal) annotation (Line(points={{2.2,14.9},{
          19.3333,14.9},{19.3333,21}},
                               color={191,0,0}));
  connect(BCWest1.thermal, WallCV1.West) annotation (Line(points={{-20.6667,9},
          {-20.6667,9.7},{-10.8,9.7}},color={191,0,0}));
  connect(BCWest2.thermal, WallCV2.West) annotation (Line(points={{-20.6667,-11},
          {-20.6667,-10.3},{-10.8,-10.3}}, color={191,0,0}));
  connect(BCWest3.thermal, WallCV3.West) annotation (Line(points={{-20.6667,-31},
          {-20.6667,-30.3},{-10.8,-30.3}}, color={191,0,0}));
  connect(BCWest4.thermal, WallCV4.West) annotation (Line(points={{-20.6667,-51},
          {-20.6667,-50.3},{-10.8,-50.3}}, color={191,0,0}));
  connect(BCWest5.thermal, WallCV5.West) annotation (Line(points={{-20.6667,-71},
          {-20.6667,-70.3},{-10.8,-70.3}}, color={191,0,0}));
  connect(WallCV1.East, BCEast1.thermal)
    annotation (Line(points={{4.8,9.7},{4.8,9},{19.3333,9}}, color={191,0,0}));
  connect(WallCV2.East, BCEast2.thermal) annotation (Line(points={{4.8,-10.3},{
          4,-11},{19.3333,-11}},
                               color={191,0,0}));
  connect(WallCV3.East, BCEast3.thermal) annotation (Line(points={{4.8,-30.3},{
          4,-31},{19.3333,-31}},
                               color={191,0,0}));
  connect(WallCV4.East, BCEast4.thermal) annotation (Line(points={{4.8,-50.3},{
          4,-51},{19.3333,-51}},
                               color={191,0,0}));
  connect(WallCV5.East, BCEast5.thermal) annotation (Line(points={{4.8,-70.3},{
          4,-71},{19.3333,-71}},
                               color={191,0,0}));
  connect(BCTop.thermal, WallCV1.North) annotation (Line(points={{-2.66667,25},{
          -3,26},{-3,13.6}}, color={191,0,0}));
  connect(BCBottom.thermal, WallCV5.South) annotation (Line(points={{-2.66667,-87},
          {-3,-86},{-3,-74.2}}, color={191,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=60, __Dymola_Algorithm="Dassl"));
end WallVertical3D;
