within DynTherM.Tests.Distributed;
model Radiation1D
  Components.HeatTransfer.SolarRadiation thermalRadiation(csi=0)
    annotation (Placement(transformation(extent={{-36,98},{36,26}})));
  inner Components.Environment environment(
    Mach_inf=0,
    Altitude(displayUnit="km") = 11000,
    ISA_plus=0,
    psi=0,
    Day=202,
    Hour=14,
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.OneDimensional.WallRadiation1D wallRadiationFlux(csi=0, N=3)
    annotation (Placement(transformation(extent={{-26,-14},{26,-80}})));
  Components.Adaptors.irradianceMultiplier irradianceMultiplier(Nx=3, Ny=1)
    annotation (Placement(transformation(extent={{-26,30},{26,-22}})));
  BoundaryConditions.thermal_flux_distributed thermal_flux_distributed(
    Nx=3,
    Ny=1,
    T=(20 + 273.15)*ones(3, 1),
    use_phi=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-36,-110},{36,-50}})));
equation
  connect(thermalRadiation.inlet, irradianceMultiplier.single)
    annotation (Line(points={{0,39.68},{0,19.6}}, color={191,0,0}));
  connect(irradianceMultiplier.distributed, wallRadiationFlux.outlet)
    annotation (Line(points={{0,-11.6},{0,-43.7}}, color={238,46,47}));
  connect(thermal_flux_distributed.thermal_flux, wallRadiationFlux.inlet)
    annotation (Line(points={{-7.10543e-15,-80},{-7.10543e-15,-68.45},{0,-68.45},
          {0,-56.9}}, color={255,127,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Radiation1D;
