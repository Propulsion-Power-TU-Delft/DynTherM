within DynTherM.Tests.MassTransfer;
model TJunction "Simple test of T-junction"

  BoundaryConditions.ZeroDimensional.pressure_source source(
    redeclare package Medium = Modelica.Media.Air.DryAirNasa,
    T_nom=293.15,
    use_in_P=true,
    use_in_T=false,
    T_start=293.15) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,0})));
  BoundaryConditions.ZeroDimensional.pressure_sink sink_orthogonal(
    redeclare package Medium = Modelica.Media.Air.DryAirNasa,
    P_di=100000,
    T_di=293.15,
    P_start=100000,
    T_start=293.15) annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=-90,
        origin={0,88})));
  Modelica.Blocks.Sources.Sine pressure_sine(
    amplitude=0.4e5,
    f=3,
    offset=1.5e5,
    startTime=0.1) annotation (Placement(transformation(extent={{-88,30},{-68,
            50}})));
  Components.MassTransfer.TJunction tJunction(
    redeclare package Medium = Modelica.Media.Air.DryAirNasa,
    V=1e-4,
    T_start=293.15,
    noInitialPressure=true)
    annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
  BoundaryConditions.ZeroDimensional.pressure_sink sink_parallel(
    redeclare package Medium = Modelica.Media.Air.DryAirNasa,
    P_di=100000,
    T_di=293.15,
    P_start=100000,
    T_start=293.15) annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=180,
        origin={88,0})));
  Components.MassTransfer.PressureDrop dP_parallel(
    redeclare package Medium = Modelica.Media.Air.DryAirNasa,
    option=DynTherM.Choices.PDropOpt.linear,
    R=1e5,
    P_start=100000,
    T_start=293.15)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Components.MassTransfer.PressureDrop dP_orthogonal(
    redeclare package Medium = Modelica.Media.Air.DryAirNasa,
    option=DynTherM.Choices.PDropOpt.linear,
    R=2e5,
    P_start=100000,
    T_start=293.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,50})));
equation
  connect(pressure_sine.y, source.in_P) annotation (Line(points={{-67,40},{
          -51.6,40},{-51.6,9.8}}, color={0,0,127}));
  connect(source.outlet, tJunction.inlet)
    annotation (Line(points={{-46,0},{-20,0}}, color={0,0,0}));
  connect(tJunction.outletOrthogonal, dP_orthogonal.inlet)
    annotation (Line(points={{0,20},{0,40}}, color={0,0,0}));
  connect(dP_orthogonal.outlet, sink_orthogonal.inlet)
    annotation (Line(points={{0,60},{0,76}}, color={0,0,0}));
  connect(tJunction.outletParallel, dP_parallel.inlet)
    annotation (Line(points={{20,0},{40,0}}, color={0,0,0}));
  connect(dP_parallel.outlet, sink_parallel.inlet)
    annotation (Line(points={{60,0},{76,0}}, color={0,0,0}));
  annotation (Documentation(info="<html>

</html>"), experiment(__Dymola_NumberOfIntervals=100, __Dymola_Algorithm=
          "Dassl"));
end TJunction;
