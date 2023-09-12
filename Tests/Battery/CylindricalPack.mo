within ThermalManagement.Tests.Battery;
model CylindricalPack
  Systems.Battery.CylindricalPack cylindricalPack(
    N_series=1,
    N_parallel=1,
    SOC=100,
    Tstart=273.15,
    initOpt=ThermalManagement.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-54,-52},{50,52}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CylindricalPack;
