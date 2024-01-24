within DynTherM.Tests.HeatTransfer;
model ConductionConvectionRadiationSteady
  "Validation test case of steady-state conduction (tube) + convection + radiation"

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
    annotation (Placement(transformation(extent={{-22,-8},{22,-50}})));
  Components.HeatTransfer.TubeConduction Insulation(
    L(displayUnit="m") = 1,
    R_ext(displayUnit="mm") = 0.15,
    R_int(displayUnit="mm") = 0.11,
    initOpt=DynTherM.Choices.InitOpt.steadyState,
    coeff=1,
    redeclare model Mat = DynTherM.Materials.GlassFibre)
    annotation (Placement(transformation(extent={{-22,24},{22,-18}})));
  Components.HeatTransfer.ExternalConvection Convection(A=A_ext,
                                                        redeclare model HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue
        (ht_fixed=20))
    annotation (Placement(transformation(extent={{20,62},{60,22}})));
  inner Components.Environment environment(
    V_inf_di=0,
    Altitude(displayUnit="m") = 2440,
    ISA_plus=5)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  BoundaryConditions.thermal thermal(
    T=673.15,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-20,-96},{10,-78}})));
  Components.HeatTransfer.InternalConvection internalConvection(A=A_int,
                                                                redeclare model
      HTC =
        DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue
        (ht_fixed=10))
    annotation (Placement(transformation(extent={{-20,-80},{20,-40}})));
  Components.HeatTransfer.WallRadiation wallRadiation(
    redeclare model Material =
        DynTherM.Materials.Paints.WhitePaint,
    A=A_ext,
    csi=0)
    annotation (Placement(transformation(extent={{-60,62},{-20,22}})));
  Components.HeatTransfer.SolarRadiation solarRadiation(csi=0)
    annotation (Placement(transformation(extent={{-62,98},{-18,54}})));
equation

  connect(Panel.outlet, Insulation.inlet) annotation (Line(points={{
          4.44089e-16,-21.86},{4.44089e-16,-4.14}},        color={191,0,0}));
  connect(Panel.inlet, internalConvection.inlet) annotation (Line(points={{
          4.44089e-16,-36.14},{4.44089e-16,-36},{0,-36},{0,-53.2},{
          4.44089e-16,-53.2}},     color={191,0,0}));
  connect(thermal.thermal, internalConvection.outlet)
    annotation (Line(points={{0,-87},{0,-66.8},{4.44089e-16,-66.8}},
                                                            color={191,0,0}));
  connect(solarRadiation.inlet,wallRadiation. outlet)
    annotation (Line(points={{-40,62.36},{-40,44.8}},  color={191,0,0}));
  connect(Insulation.outlet, wallRadiation.inlet) annotation (Line(points={
          {4.44089e-16,10.14},{4.44089e-16,20},{-40,20},{-40,35.2}}, color=
          {191,0,0}));
  connect(Convection.inlet, wallRadiation.inlet) annotation (Line(points={{
          40,35.2},{40,20},{-40,20},{-40,35.2}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=100, Interval=0.1));
end ConductionConvectionRadiationSteady;
