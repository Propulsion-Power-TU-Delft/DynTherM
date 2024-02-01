within DynTherM.Tests.HeatTransfer;
model WallConduction
  Components.HeatTransfer.WallConduction wallConduction(
    t(displayUnit="mm") = 0.001,
    A=1,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-34,-24},{34,24}})));
  CustomInterfaces.Adaptors.heatFluxToHeatFlow heatFluxToHeatFlow(A=1)
    annotation (Placement(transformation(extent={{-28,76},{28,20}})));
  BoundaryConditions.thermal_flux thermal_flux(phi=-13800)
    annotation (Placement(transformation(extent={{-24,78},{12,102}})));
  CustomInterfaces.Adaptors.heatFluxToHeatFlow heatFluxToHeatFlow1(A=1)
    annotation (Placement(transformation(extent={{-28,-76},{28,-20}})));
  BoundaryConditions.thermal_flux thermal_flux1(
    T=323.15,
    phi=-13800,
    use_phi=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-24,-102},{12,-78}})));
equation
  connect(heatFluxToHeatFlow.inlet, wallConduction.inlet) annotation (Line(
        points={{3.55271e-15,31.2},{3.55271e-15,19.68},{0,19.68},{0,8.16}},
        color={191,0,0}));
  connect(thermal_flux.thermal_flux, heatFluxToHeatFlow.outlet) annotation (
      Line(points={{0,90},{0,77.4},{3.55271e-15,77.4},{3.55271e-15,64.8}},
        color={255,127,0}));
  connect(wallConduction.outlet, heatFluxToHeatFlow1.inlet) annotation (Line(
        points={{0,-8.16},{0,-19.68},{3.55271e-15,-19.68},{3.55271e-15,-31.2}},
        color={191,0,0}));
  connect(thermal_flux1.thermal_flux, heatFluxToHeatFlow1.outlet) annotation (
      Line(points={{0,-90},{0,-77.4},{3.55271e-15,-77.4},{3.55271e-15,-64.8}},
        color={255,127,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end WallConduction;
