within DynTherM.Tests.Distributed;
model ExternalConvection
  parameter Length c=5 "Airfoil chord";
  parameter Length t_skin=3e-3 "Skin thickness";
  parameter Area A_ht=1 "Total heat transfer area";

  Components.TwoDimensional.ExternalConvection2D Conv2D(
    redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.Wing
        (x=c, V=environment.V_inf),
    A=A_ht,
    Nx=1,
    Ny=1) annotation (Placement(transformation(extent={{62,52},{98,16}})));
  Components.OneDimensional.ExternalConvection1D Conv1D(
    redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.Wing
        (x=c, V=environment.V_inf),
    A=A_ht,
    N=10)
         annotation (Placement(transformation(extent={{-18,52},{18,16}})));
  Components.HeatTransfer.ExternalConvection Conv0D(A=A_ht, redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.Wing
        (x=c, V=environment.V_inf))
    annotation (Placement(transformation(extent={{-98,52},{-62,16}})));
  BoundaryConditions.ZeroDimensional.thermal ZeroDimBC(
    use_Q=false,
    use_T=false,
    use_in_T=false,
    use_in_Q=true)
    annotation (Placement(transformation(extent={{-68,-46},{-86,-34}})));
  BoundaryConditions.OneDimensional.thermal1D OneDimBC(
    Nx=Conv1D.N,
    use_di_Q=false,
    use_di_T=false,
    use_in_Q=true)
    annotation (Placement(transformation(extent={{-26,-24},{26,-56}})));
  BoundaryConditions.TwoDimensional.thermal2D TwoDimBC(
    Nx=Conv2D.Nx,
    Ny=Conv2D.Ny,
    use_di_Q=false,
    use_di_T=false,
    use_in_Q=true)
    annotation (Placement(transformation(extent={{54,-24},{106,-56}})));
  inner Components.Environment environment(
    phi_amb=0.2,
    phi_amb_ground=0.2,
    use_di_altitude=false,
    use_di_Mach_inf=false,
    use_di_V_inf=false,
    use_in_altitude=true,
    use_in_Mach_inf=true)
    annotation (Placement(transformation(extent={{62,62},{94,94}})));
  Modelica.Blocks.Sources.Ramp Mach_inf(
    height=0.565,
    duration(displayUnit="min") = 1380,
    offset=0.05)
    annotation (Placement(transformation(extent={{22,82},{38,98}})));
  Modelica.Blocks.Sources.Ramp Altitude(
    height=7620,
    duration(displayUnit="min") = 1200,
    startTime(displayUnit="min") = 180)
    annotation (Placement(transformation(extent={{22,52},{38,68}})));
  Components.HeatTransfer.WallConduction Skin0D(t=t_skin, A=A_ht,
    Tstart=323.15)
    annotation (Placement(transformation(extent={{-98,10},{-62,-10}})));
  Components.TwoDimensional.WallConductionHorizontal2D Skin1D(
    x=c,
    y=t_skin,
    z=A_ht/c,
    Tstart=323.15,
    N=Conv1D.N)
    annotation (Placement(transformation(extent={{-18,16},{18,-16}})));
  Modelica.Blocks.Sources.Ramp Q_int(
    height=-3e3,
    duration(displayUnit="min") = 1200,
    offset=5e3,
    startTime(displayUnit="min") = 180)
    annotation (Placement(transformation(extent={{98,-88},{82,-72}})));
  BoundaryConditions.ZeroDimensional.thermal AdiabaticBC0D(
    Q=0,
    use_Q=true,
    use_T=false,
    use_in_T=false,
    use_in_Q=false)
    annotation (Placement(transformation(extent={{28,16},{16,24}})));
  Components.ThreeDimensional.Conduction3D Skin2D(
    x=c,
    y=t_skin,
    z=A_ht/c,
    Tstart=323.15,
    Nx=Conv2D.Nx,
    Ny=1,
    Nz=Conv2D.Ny)
    annotation (Placement(transformation(extent={{62,-18},{98,18}})));
  BoundaryConditions.TwoDimensional.thermal2D AdiabaticBC2D(
    Nx=Conv2D.Nx,
    Ny=Conv2D.Ny,
    Q=zeros(Conv2D.Nx, Conv2D.Ny),
    use_di_Q=true,
    use_di_T=false,
    use_in_Q=false)
    annotation (Placement(transformation(extent={{48,30},{72,10}})));
equation
  connect(Mach_inf.y, environment.in_Mach_inf) annotation (Line(points={{38.8,90},
          {50,90},{50,78},{62,78}}, color={0,0,127}));
  connect(Altitude.y, environment.in_altitude) annotation (Line(points={{38.8,60},
          {50,60},{50,71.6},{62,71.6}}, color={0,0,127}));
  connect(Conv0D.inlet, Skin0D.outlet)
    annotation (Line(points={{-80,27.88},{-80,3.4}}, color={191,0,0}));
  connect(Skin0D.inlet, ZeroDimBC.thermal)
    annotation (Line(points={{-80,-3.4},{-80,-40}}, color={191,0,0}));
  connect(Q_int.y, TwoDimBC.in_Q) annotation (Line(points={{81.2,-80},{71.3333,
          -80},{71.3333,-46.4}},
                            color={0,0,127}));
  connect(Q_int.y, ZeroDimBC.in_Q) annotation (Line(points={{81.2,-80},{-60,-80},
          {-60,-46},{-66.2,-46}}, color={0,0,127}));
  connect(Conv1D.inlet, Skin1D.South)
    annotation (Line(points={{0,28.6},{0,4.8}}, color={191,0,0}));
  connect(Skin1D.North, OneDimBC.thermal)
    annotation (Line(points={{0,-4.8},{0,-40}}, color={191,0,0}));
  connect(AdiabaticBC0D.thermal, Skin1D.East)
    annotation (Line(points={{20,20},{20,0},{16.2,0}}, color={191,0,0}));
  connect(AdiabaticBC0D.thermal, Skin1D.West)
    annotation (Line(points={{20,20},{-16.2,20},{-16.2,0}}, color={191,0,0}));
  connect(Q_int.y, OneDimBC.in_Q) annotation (Line(points={{81.2,-80},{-8.66667,
          -80},{-8.66667,-46.4}}, color={0,0,127}));
  connect(AdiabaticBC2D.thermal, Skin2D.West)
    annotation (Line(points={{60,20},{60,0},{71,0}}, color={191,0,0}));
  connect(AdiabaticBC2D.thermal, Skin2D.Rear)
    annotation (Line(points={{60,20},{60,10.8},{89,10.8}}, color={191,0,0}));
  connect(AdiabaticBC2D.thermal, Skin2D.Front)
    annotation (Line(points={{60,20},{60,-10.8},{71,-10.8}}, color={191,0,0}));
  connect(AdiabaticBC2D.thermal, Skin2D.East)
    annotation (Line(points={{60,20},{60,0},{89,0}}, color={191,0,0}));
  connect(TwoDimBC.thermal, Skin2D.South)
    annotation (Line(points={{80,-40},{80,-3.6}}, color={191,0,0}));
  connect(Skin2D.North, Conv2D.inlet)
    annotation (Line(points={{80,3.6},{80,28.6}}, color={191,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=1380,
      Interval=1,
      __Dymola_Algorithm="Dassl"));
end ExternalConvection;
