within DynTherM.Tests.HeatTransfer;
model WindowRadation
  inner Components.Environment environment(
    V_inf=0,
    Altitude(displayUnit="km") = 0,
    ISA_plus=0)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.HeatTransfer.SolarRadiation thermalRadiation(csi=0)
             annotation (Placement(transformation(extent={{-36,96},{36,24}})));
  Components.HeatTransfer.WindowRadiation windowRadiation(A=1.9)
    annotation (Placement(transformation(extent={{-24,26},{24,-22}})));
  BoundaryConditions.thermal int(
    T=293.15,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{0,-70},{30,-52}})));
  CustomInterfaces.IrradiancePort irradiancePort annotation (Placement(
        transformation(extent={{-24,-64},{-16,-56}}), iconTransformation(
          extent={{-24,-64},{-16,-56}})));
  Components.HeatTransfer.WallConduction outerLayerConduction(
    redeclare model Mat = Materials.Opticor,
    t(displayUnit="mm") = 0.01,
    A=0.33*0.23,
    Tstart=263.15,
    initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{-4,-56},{44,-16}})));
equation
  connect(thermalRadiation.inlet, windowRadiation.outlet)
    annotation (Line(points={{0,37.68},{0,5.36}}, color={191,0,0}));
  connect(irradiancePort, windowRadiation.inlet_transmitted) annotation (
      Line(points={{-20,-60},{-20,-32},{-8.16,-32},{-8.16,-6.16}}, color={
          191,0,0}));
  connect(windowRadiation.inlet_absorbed, outerLayerConduction.inlet)
    annotation (Line(points={{8.16,-6.16},{8.16,-29.2},{20,-29.2}}, color={
          191,0,0}));
  connect(outerLayerConduction.outlet, int.thermal)
    annotation (Line(points={{20,-42.8},{20,-61}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end WindowRadation;
