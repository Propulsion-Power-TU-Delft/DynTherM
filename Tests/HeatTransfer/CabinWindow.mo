within DynTherM.Tests.HeatTransfer;
model CabinWindow
  Components.HeatTransfer.SolarRadiation solarRadiation(csi=0.5235987755983)
    annotation (Placement(transformation(extent={{-36,94},{18,40}})));
  inner Components.Environment environment(
    phi_amb=0.2,
    allowFlowReversal=true,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{-100,60},{-60,100}})));
  Systems.Aircraft.Subsystems.CabinWindow cabinWindow(
    t_outer(displayUnit="mm") = 0.01,
    t_inner(displayUnit="mm") = 0.005,
    H_window=0.33,
    L_window=0.23,
    Tstart=283.15)
    annotation (Placement(transformation(extent={{-46,-48},{46,32}})));
  BoundaryConditions.ZeroDimensional.thermal int(
    T=297.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{0,-90},{30,-70}})));
  BoundaryConditions.ZeroDimensional.thermal int1(
    T=297.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-40,-90},{-10,-70}})));
equation
  cabinWindow.P_air=85e3;
  cabinWindow.X_air=environment.X_amb;
  connect(solarRadiation.inlet, cabinWindow.irradianceExt) annotation (Line(
        points={{-9,50.26},{-9,48},{-9.2,48},{-9.2,18.6667}}, color={191,0,0}));
  connect(cabinWindow.heatTransmitted, int1.thermal) annotation (Line(points={{
          -9.2,-16.8889},{-9.2,-40},{-20,-40},{-20,-80}}, color={191,0,0}));
  connect(cabinWindow.heatAbsorbed, int.thermal) annotation (Line(points={{9.2,
          -16.8889},{9.2,-40},{20,-40},{20,-80}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CabinWindow;
