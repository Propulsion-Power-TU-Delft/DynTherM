within DynTherM.Tests.HeatTransfer;
model ConductionConvectionUnsteady
  "Validation test case of unsteady conduction (tube) + convection"

  constant Real pi=Modelica.Constants.pi;
  final parameter Modelica.Units.SI.Area A_int=Panel.coeff*2*pi*Panel.L*Panel.R_int "Internal area";
  final parameter Modelica.Units.SI.Area A_ext=Insulation.coeff*2*pi*Insulation.L*Insulation.R_ext "External area";

  Components.HeatTransfer.TubeConduction Panel(
    L(displayUnit="m") = 1,
    R_ext(displayUnit="mm") = 0.11,
    R_int(displayUnit="mm") = 0.105,
    initOpt=DynTherM.Choices.InitOpt.steadyState,
    coeff=1,
    redeclare model Mat = Materials.Aluminium)
    annotation (Placement(transformation(extent={{-28,2},{28,-42}})));
  Components.HeatTransfer.TubeConduction Insulation(
    L(displayUnit="m") = 1,
    R_ext(displayUnit="mm") = 0.15,
    R_int(displayUnit="mm") = 0.11,
    initOpt=DynTherM.Choices.InitOpt.steadyState,
    coeff=1,
    redeclare model Mat = DynTherM.Materials.GlassFibre)
    annotation (Placement(transformation(extent={{-28,42},{28,-2}})));
  Components.HeatTransfer.ExternalConvection Convection(
    Nx=3,                                               A=A_ext,
    redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue
        (ht_fixed=20*ones(3, 1)))
    annotation (Placement(transformation(extent={{-28,110},{28,56}})));
  inner Components.Environment environment(
    V_inf_di=0,
    ISA_plus=5)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  BoundaryConditions.thermal thermal(
    T=673.15,
    use_Q=false,
    use_T=false,
    use_in_T=true)
    annotation (Placement(transformation(extent={{-20,-96},{10,-78}})));
  Components.HeatTransfer.InternalConvection internalConvection(A=A_int,
                                                                redeclare model
      HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue
        (ht_fixed=10))
    annotation (Placement(transformation(extent={{-24,-82},{24,-34}})));
  Modelica.Blocks.Sources.Step step(
    height=100,
    offset=573.15,
    startTime=10)
    annotation (Placement(transformation(extent={{-70,-88},{-50,-68}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier(Nx=3, Ny=1)
    annotation (Placement(transformation(extent={{-18,34},{18,70}})));
equation

  connect(Panel.outlet, Insulation.inlet) annotation (Line(points={{3.9968e-15,-12.52},
          {3.9968e-15,-0.26},{0,-0.26},{0,12.52}},         color={191,0,0}));
  connect(Panel.inlet, internalConvection.inlet) annotation (Line(points={{3.55271e-15,
          -27.48},{3.55271e-15,-36},{4.44089e-16,-36},{4.44089e-16,-49.84}},
                                   color={191,0,0}));
  connect(thermal.thermal, internalConvection.outlet)
    annotation (Line(points={{0,-87},{0,-84},{4.44089e-16,-84},{4.44089e-16,-66.16}},
                                                            color={191,0,0}));
  connect(step.y, thermal.in_T)
    annotation (Line(points={{-49,-78},{-23,-78}}, color={0,0,127}));
  connect(Convection.inlet, heatFlowMultiplier.distributed) annotation (Line(
        points={{3.55271e-15,75.44},{3.55271e-15,69.12},{0,69.12},{0,62.8}},
        color={191,0,0}));
  connect(heatFlowMultiplier.single, Insulation.outlet) annotation (Line(points
        ={{0,41.2},{0,34.34},{3.55271e-15,34.34},{3.55271e-15,27.48}}, color={
          191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=100, Interval=0.1));
end ConductionConvectionUnsteady;
