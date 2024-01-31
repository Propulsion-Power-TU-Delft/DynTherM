within DynTherM.Tests.Distributed;
model CircularCV
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1)
    "Refrigerant";

  BoundaryConditions.flow_source ECSFlow(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,0})));
  BoundaryConditions.pressure_sink pressureSink(redeclare package Medium =
        Refrigerant)
    annotation (Placement(transformation(extent={{68,-12},{92,12}})));
  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.steadyState) annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.OneDimensional.CircularCV cv(
    redeclare package Medium = Refrigerant,
    L(displayUnit="mm") = 0.4826,
    R_ext(displayUnit="mm") = 0.003,
    R_int(displayUnit="mm") = 0.0025,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{-30,-30},{30,30}})));

  BoundaryConditions.thermal thermal(Q(displayUnit="kW") = -10000)
    annotation (Placement(transformation(extent={{-16,52},{8,68}})));
  Modelica.Blocks.Sources.Constant m(k=0.3)
    annotation (Placement(transformation(extent={{-100,20},{-80,40}})));
  Modelica.Blocks.Sources.Constant T(k=273.15 + 50)
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
equation

  connect(ECSFlow.outlet, cv.inlet) annotation (Line(points={{-46,-3.44169e-15},
          {-45,-3.44169e-15},{-45,0},{-24,0}}, color={0,0,0}));
  connect(cv.outlet, pressureSink.inlet)
    annotation (Line(points={{24,0},{68,0}}, color={0,0,0}));
  connect(thermal.thermal, cv.solid_surface)
    annotation (Line(points={{0,60},{0,24}}, color={191,0,0}));
  connect(m.y, ECSFlow.in_massFlow) annotation (Line(points={{-79,30},{-71.2,30},
          {-71.2,9.8}}, color={0,0,127}));
  connect(T.y, ECSFlow.in_T) annotation (Line(points={{-79,70},{-62.8,70},{-62.8,
          9.8}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CircularCV;
