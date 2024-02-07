within DynTherM.Tests.HeatTransfer;
model ConductionConvectionSteady
  "Validation test case of steady-state conduction (tube) + convection"

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
    A=A_ext, redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue
        (ht_fixed=20*ones(1, 1)))
    annotation (Placement(transformation(extent={{-28,96},{28,42}})));
  inner Components.Environment environment(
    V_inf_di=0,
    ISA_plus=5)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  BoundaryConditions.thermal_distributed thermal(
    Nx=1,
    Ny=1,
    T(displayUnit="K") = 400*ones(1, 1),
    use_di_Q=false,
    use_di_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-18,-102},{18,-78}})));
  Components.HeatTransfer.InternalConvection internalConvection(A=A_int,
      redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue
        (ht_fixed=10*ones(1, 1)))
    annotation (Placement(transformation(extent={{-24,-82},{24,-34}})));
equation

  connect(thermal.thermal, internalConvection.outlet)
    annotation (Line(points={{0,-90},{0,-84},{4.44089e-16,-84},{4.44089e-16,
          -62.8}},                                          color={191,0,0}));
  connect(Convection.inlet, Insulation.outlet) annotation (Line(points={{3.55271e-15,
          61.44},{3.55271e-15,60},{0,60},{0,26.16},{3.55271e-15,26.16}}, color={
          191,0,0}));
  connect(Insulation.inlet, Panel.outlet) annotation (Line(points={{3.55271e-15,
          13.84},{3.55271e-15,-13.84}}, color={191,0,0}));
  connect(Panel.inlet, internalConvection.inlet) annotation (Line(points={{3.55271e-15,
          -26.16},{3.55271e-15,-39.68},{0,-39.68},{0,-53.2}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=100, Interval=0.1));
end ConductionConvectionSteady;
