within DynTherM.Tests.HeatTransfer;
model ConductionSphereUnsteady
  "Validation test case of unsteady conduction in a hollow sphere"
  Components.HeatTransfer.SphereConduction sphereConduction(
    redeclare model Mat = DynTherM.Materials.Aluminium,
    R_ext=0.6,
    R_int=0.1,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{-40,24},{40,-24}})));
  BoundaryConditions.thermal cold_side(
    T=283.15,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-20,-68},{10,-50}})));
  BoundaryConditions.thermal hot_side(
    T=373.15,
    use_T=false,
    use_in_T=true)
    annotation (Placement(transformation(extent={{-20,50},{10,68}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=100,
    duration=50,
    offset=273.15,
    startTime=10)
    annotation (Placement(transformation(extent={{-72,58},{-52,78}})));
equation
  connect(hot_side.thermal, sphereConduction.outlet)
    annotation (Line(points={{0,59},{0,8.16}}, color={191,0,0}));
  connect(cold_side.thermal, sphereConduction.inlet)
    annotation (Line(points={{0,-59},{0,-8.16}}, color={191,0,0}));
  connect(ramp.y, hot_side.in_T)
    annotation (Line(points={{-51,68},{-23,68}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=3000, Interval=0.1));
end ConductionSphereUnsteady;
