within ThermalManagement.Tests.HeatTransfer;
model HeatCapacity
  Components.HeatTransfer.HeatCapacity heatCapacity(
    initOpt=ThermalManagement.Choices.InitOpt.fixedState,
    m=20*180,
    c=1000,
    A=2*180)
    annotation (Placement(transformation(extent={{-28,-28},{28,28}})));
  Components.HeatTransfer.SolarRadiation solarRadiation(csi=0.5235987755983)
    annotation (Placement(transformation(extent={{-30,96},{30,36}})));
  BoundaryConditions.thermal int(
    T=297.15,
    use_T=false,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-20,-88},{10,-70}})));
  inner Components.Environment environment(
    Altitude(displayUnit="km") = 11000,
    phi_amb=0.2,
    allowFlowReversal=true,
    initOpt=ThermalManagement.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{-100,60},{-60,100}})));
equation
  connect(heatCapacity.irradiancePort, solarRadiation.inlet) annotation (
      Line(points={{3.55271e-15,-1.12},{3.55271e-15,23.14},{0,23.14},{0,
          47.4}}, color={191,0,0}));
  connect(int.thermal, heatCapacity.heatPort) annotation (Line(points={{0,
          -79},{0,-53.5},{3.55271e-15,-53.5},{3.55271e-15,-28}}, color={191,
          0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=1000, __Dymola_Algorithm="Dassl"));
end HeatCapacity;
