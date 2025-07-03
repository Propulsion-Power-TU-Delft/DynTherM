within DynTherM.Tests.Distributed;
model InternalConvection2D
  Components.TwoDimensional.InternalConvection2D internalConvection2D(
    redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue
        (ht_fixed=20),
    A=1,
    Nx=3,
    Ny=1) annotation (Placement(transformation(extent={{-22,28},{22,72}})));
  Components.HeatTransfer.InternalConvection internalConvection1(A=1/3,
      redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue
        (ht_fixed=20))
    annotation (Placement(transformation(extent={{-52,-52},{-28,-28}})));
  Components.HeatTransfer.InternalConvection internalConvection2(A=1/3,
      redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue
        (ht_fixed=20))
    annotation (Placement(transformation(extent={{-12,-52},{12,-28}})));
  Components.HeatTransfer.InternalConvection internalConvection3(A=1/3,
      redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue
        (ht_fixed=20))
    annotation (Placement(transformation(extent={{28,-52},{52,-28}})));
  BoundaryConditions.ZeroDimensional.thermal TopBC1(
    T=323.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-48,-24},{-36,-16}})));
  BoundaryConditions.ZeroDimensional.thermal TopBC2(
    T=373.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-8,-24},{4,-16}})));
  BoundaryConditions.ZeroDimensional.thermal TopBC3(
    T=323.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{32,-24},{44,-16}})));
  BoundaryConditions.ZeroDimensional.thermal BottomBC1(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-48,-64},{-36,-56}})));
  BoundaryConditions.ZeroDimensional.thermal BottomBC2(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-8,-64},{4,-56}})));
  BoundaryConditions.ZeroDimensional.thermal BottomBC3(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{32,-64},{44,-56}})));
  BoundaryConditions.TwoDimensional.thermal2D BCTop(
    Nx=internalConvection2D.Nx,
    Ny=internalConvection2D.Ny,
    T={{323.15},{373.15},{323.15}},
    use_di_Q=false,
    use_di_T=true)
    annotation (Placement(transformation(extent={{-22,70},{22,88}})));
  BoundaryConditions.TwoDimensional.thermal2D BCBottom(
    Nx=internalConvection2D.Nx,
    Ny=internalConvection2D.Ny,
    T={{298.15},{298.15},{298.15}},
    use_di_Q=false,
    use_di_T=true)
    annotation (Placement(transformation(extent={{-22,12},{22,30}})));
equation
  connect(TopBC1.thermal, internalConvection1.inlet)
    annotation (Line(points={{-40,-20},{-40,-35.92}}, color={191,0,0}));
  connect(internalConvection1.outlet, BottomBC1.thermal)
    annotation (Line(points={{-40,-44.08},{-40,-60}}, color={191,0,0}));
  connect(TopBC2.thermal, internalConvection2.inlet)
    annotation (Line(points={{0,-20},{0,-35.92}}, color={191,0,0}));
  connect(internalConvection2.outlet, BottomBC2.thermal)
    annotation (Line(points={{0,-44.08},{0,-60}}, color={191,0,0}));
  connect(TopBC3.thermal, internalConvection3.inlet)
    annotation (Line(points={{40,-20},{40,-35.92}}, color={191,0,0}));
  connect(internalConvection3.outlet, BottomBC3.thermal)
    annotation (Line(points={{40,-44.08},{40,-60}}, color={191,0,0}));
  connect(BCTop.thermal, internalConvection2D.inlet)
    annotation (Line(points={{0,79},{0,56.6}}, color={191,0,0}));
  connect(internalConvection2D.outlet, BCBottom.thermal)
    annotation (Line(points={{0,43.4},{0,21}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end InternalConvection2D;
