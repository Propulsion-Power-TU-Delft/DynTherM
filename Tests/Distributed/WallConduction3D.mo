within DynTherM.Tests.Distributed;
model WallConduction3D
  Components.ThreeDimensional.Conduction3D wallConduction3D(
    x(displayUnit="mm") = 0.1,
    y(displayUnit="mm") = 0.035,
    z=1,
    Nx=5,
    Ny=3,
    Nz=10) annotation (Placement(transformation(extent={{-40,-40},{40,40}})));
  BoundaryConditions.TwoDimensional.thermal2D WestBC(
    Nx=wallConduction3D.Ny,
    Ny=wallConduction3D.Nz,
    T=323.15*ones(wallConduction3D.Ny, wallConduction3D.Nz),
                                                     use_di_Q=false, use_di_T=
        true) annotation (Placement(transformation(
        extent={{-16,-8},{16,8}},
        rotation=90,
        origin={-60,0})));
  BoundaryConditions.TwoDimensional.thermal2D EastBC(
    Nx=wallConduction3D.Ny,
    Ny=wallConduction3D.Nz,
    T=298.15*ones(wallConduction3D.Ny, wallConduction3D.Nz),
                                                     use_di_Q=false, use_di_T=
        true) annotation (Placement(transformation(
        extent={{-16,8},{16,-8}},
        rotation=90,
        origin={60,0})));
  BoundaryConditions.TwoDimensional.thermal2D NorthBC(
    Nx=wallConduction3D.Nx,
    Ny=wallConduction3D.Nz,
    T=298.15*ones(wallConduction3D.Nx, wallConduction3D.Nz),
                                                      use_di_Q=false, use_di_T=
        true) annotation (Placement(transformation(
        extent={{-16,8},{16,-8}},
        rotation=180,
        origin={0,60})));
  BoundaryConditions.TwoDimensional.thermal2D SouthBC(
    Nx=wallConduction3D.Nx,
    Ny=wallConduction3D.Nz,
    T=323.15*ones(wallConduction3D.Nx, wallConduction3D.Nz),
                                                      use_di_Q=false, use_di_T=
        true) annotation (Placement(transformation(
        extent={{-16,-8},{16,8}},
        rotation=180,
        origin={0,-60})));
  BoundaryConditions.TwoDimensional.thermal2D RearBC(
    Nx=wallConduction3D.Nx,
    Ny=wallConduction3D.Ny,
    T=298.15*ones(wallConduction3D.Nx, wallConduction3D.Ny),
                                                     use_di_Q=false, use_di_T=
        true) annotation (Placement(transformation(
        extent={{-16,8},{16,-8}},
        rotation=180,
        origin={20,80})));
  BoundaryConditions.TwoDimensional.thermal2D FrontBC(
    Nx=wallConduction3D.Nx,
    Ny=wallConduction3D.Ny,
    T=323.15*ones(wallConduction3D.Nx, wallConduction3D.Ny),
                                                      use_di_Q=false, use_di_T=
        true) annotation (Placement(transformation(
        extent={{-16,-8},{16,8}},
        rotation=180,
        origin={-20,-80})));
equation
  connect(WestBC.thermal, wallConduction3D.West)
    annotation (Line(points={{-60,0},{-20,0}}, color={191,0,0}));
  connect(wallConduction3D.East, EastBC.thermal)
    annotation (Line(points={{20,0},{60,0}}, color={191,0,0}));
  connect(wallConduction3D.North, NorthBC.thermal)
    annotation (Line(points={{0,8},{0,60}}, color={191,0,0}));
  connect(SouthBC.thermal, wallConduction3D.South)
    annotation (Line(points={{0,-60},{0,-8}}, color={191,0,0}));
  connect(wallConduction3D.Rear, RearBC.thermal)
    annotation (Line(points={{20,24},{20,80}}, color={191,0,0}));
  connect(FrontBC.thermal, wallConduction3D.Front)
    annotation (Line(points={{-20,-80},{-20,-24}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=600,
      __Dymola_NumberOfIntervals=600,
      __Dymola_Algorithm="Dassl"));
end WallConduction3D;
