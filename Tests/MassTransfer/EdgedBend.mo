within DynTherM.Tests.MassTransfer;
model EdgedBend "Simple test of edged bend"

  BoundaryConditions.ZeroDimensional.pressure_source source(
    redeclare package Medium = Modelica.Media.Air.DryAirNasa,
    T_nom=293.15,
    use_in_P=true,
    use_in_T=false,
    T_start=293.15) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,0})));
  BoundaryConditions.ZeroDimensional.pressure_sink sink(
    redeclare package Medium = Modelica.Media.Air.DryAirNasa,
    P_di=100000,
    T_di=293.15,
    P_start=100000,
    T_start=293.15) annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=-90,
        origin={0,60})));
  Components.MassTransfer.EdgedBend edgedBend(
    redeclare package Medium = Modelica.Media.Air.DryAirNasa,
    D=1e-2,
    delta=1.5707963267949,
    ks=1e-6) annotation (Placement(transformation(extent={{-14,-14},{14,14}})));
  Modelica.Blocks.Sources.Sine pressure_sine(
    amplitude=0.4e5,
    f=3,
    offset=1.5e5,
    startTime=0.1) annotation (Placement(transformation(extent={{-88,30},{-68,
            50}})));
equation
  connect(pressure_sine.y, source.in_P) annotation (Line(points={{-67,40},{
          -51.6,40},{-51.6,9.8}}, color={0,0,127}));
  connect(source.outlet, edgedBend.inlet)
    annotation (Line(points={{-46,0},{-14,0}}, color={0,0,0}));
  connect(edgedBend.outlet, sink.inlet)
    annotation (Line(points={{0,14},{0,48}}, color={0,0,0}));
  annotation (Documentation(info="<html>

</html>"), experiment(__Dymola_NumberOfIntervals=100, __Dymola_Algorithm=
          "Dassl"));
end EdgedBend;
