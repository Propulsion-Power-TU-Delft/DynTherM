within DynTherM.Tests.Distributed;
model ParallelPlatesRadiation2D
  parameter Area A=1 "Heat transfer area (total)" annotation (Dialog(enable=true));
  parameter Real eps_in(min=0,max=1)=0.1 "Emissivity of inlet surface: 1 is black body, 0 is pure reflection" annotation (Dialog(enable=true));
  parameter Real eps_out(min=0,max=1)=0.2 "Emissivity of outlet surface: 1 is black body, 0 is pure reflection" annotation (Dialog(enable=true));
  final parameter Real Gr(unit="m2")=A/3/(1/eps_in + 1/eps_out - 1);
  Components.TwoDimensional.ParallelPlatesRadiation2D parallelPlatesRadiation2D(
    eps_in=eps_in,
    eps_out=eps_out,
    A=A,
    Nx=3,
    Ny=1) annotation (Placement(transformation(extent={{-20,30},{20,70}})));
  Modelica.Thermal.HeatTransfer.Components.BodyRadiation radiation1(Gr=Gr)
                                                                    annotation (
     Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=90,
        origin={-40,-40})));
  Modelica.Thermal.HeatTransfer.Components.BodyRadiation radiation2(Gr=Gr)
                                                                    annotation (
     Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=90,
        origin={0,-40})));
  Modelica.Thermal.HeatTransfer.Components.BodyRadiation radiation3(Gr=Gr)
                                                                    annotation (
     Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=90,
        origin={40,-40})));
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
    Nx=parallelPlatesRadiation2D.Nx,
    Ny=parallelPlatesRadiation2D.Ny,
    T={{323.15},{373.15},{323.15}},
    use_di_Q=false,
    use_di_T=true)
    annotation (Placement(transformation(extent={{-22,70},{22,88}})));
  BoundaryConditions.TwoDimensional.thermal2D BCBottom(
    Nx=parallelPlatesRadiation2D.Nx,
    Ny=parallelPlatesRadiation2D.Ny,
    T={{298.15},{298.15},{298.15}},
    use_di_Q=false,
    use_di_T=true)
    annotation (Placement(transformation(extent={{-22,12},{22,30}})));
equation
  connect(BCTop.thermal, parallelPlatesRadiation2D.inlet)
    annotation (Line(points={{0,79},{0,63.6}}, color={191,0,0}));
  connect(parallelPlatesRadiation2D.outlet, BCBottom.thermal)
    annotation (Line(points={{0,36.4},{0,21}}, color={191,0,0}));
  connect(TopBC1.thermal, radiation1.port_a)
    annotation (Line(points={{-40,-20},{-40,-28}}, color={191,0,0}));
  connect(radiation1.port_b, BottomBC1.thermal)
    annotation (Line(points={{-40,-52},{-40,-60}}, color={191,0,0}));
  connect(TopBC2.thermal, radiation2.port_a)
    annotation (Line(points={{0,-20},{0,-28}}, color={191,0,0}));
  connect(radiation2.port_b, BottomBC2.thermal)
    annotation (Line(points={{0,-52},{0,-60}}, color={191,0,0}));
  connect(TopBC3.thermal, radiation3.port_a)
    annotation (Line(points={{40,-20},{40,-28}}, color={191,0,0}));
  connect(radiation3.port_b, BottomBC3.thermal)
    annotation (Line(points={{40,-52},{40,-60}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ParallelPlatesRadiation2D;
