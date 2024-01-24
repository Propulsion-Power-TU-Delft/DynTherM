within DynTherM.Tests.HeatTransfer;
model RadiationFlux
  Components.HeatTransfer.SolarRadiation thermalRadiation(csi=0)
    annotation (Placement(transformation(extent={{-36,98},{36,26}})));
  inner Components.Environment environment(
    V_inf_di=0,
    Altitude(displayUnit="km") = 11000,
    ISA_plus=0,
    psi=0,
    Day=202,
    Hour=14,
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));

  Components.HeatTransfer.WallRadiationFlux wallRadiationFlux(csi=0)
    annotation (Placement(transformation(extent={{-26,28},{26,-24}})));
  BoundaryConditions.thermal_flux thermal_flux(
    T=293.15,
    use_phi=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-20,-50},{10,-30}})));
equation
  connect(wallRadiationFlux.outlet, thermalRadiation.inlet)
    annotation (Line(points={{0,5.64},{0,39.68}}, color={191,0,0}));
  connect(thermal_flux.thermal_flux, wallRadiationFlux.inlet) annotation (Line(
        points={{0,-40},{0,-6.84},{4.44089e-16,-6.84}}, color={255,127,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RadiationFlux;
