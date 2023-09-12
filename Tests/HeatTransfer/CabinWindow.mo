within DynTherM.Tests.HeatTransfer;
model CabinWindow
  Components.HeatTransfer.SolarRadiation solarRadiation(csi=0.5235987755983)
    annotation (Placement(transformation(extent={{-22,98},{22,54}})));
  inner Components.Environment environment(
    Altitude(displayUnit="km") = 11000,
    phi_amb=0.2,
    allowFlowReversal=true,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{-100,60},{-60,100}})));
  CustomInterfaces.IrradiancePort irradiancePort
    annotation (Placement(transformation(extent={{-50,-90},{-30,-70}})));
  Systems.Aircraft.Subsystems.CabinWindow cabinWindow(
    t_outer(displayUnit="mm") = 0.01,
    t_inner(displayUnit="mm") = 0.005,
    h_window=0.33,
    l_window=0.23,
    w_air_gap(displayUnit="mm") = 0.007,
    Tstart=283.15)
    annotation (Placement(transformation(extent={{-40,-40},{40,40}})));
  BoundaryConditions.thermal int(
    T=297.15,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{20,-90},{50,-72}})));
equation
  cabinWindow.P_air=85e3;
  cabinWindow.X_air=environment.X_amb;
  connect(cabinWindow.irradianceExt, solarRadiation.inlet)
    annotation (Line(points={{-8,26.6667},{-8,44},{0,44},{0,62.36}},
                                                color={191,0,0}));
  connect(cabinWindow.irradianceInt, irradiancePort) annotation (Line(points={{-8,
          -8.88889},{-8,-50},{-40,-50},{-40,-80}},
                                              color={191,0,0}));
  connect(int.thermal, cabinWindow.heatAbsorbed) annotation (Line(points={{
          40,-81},{40,-50},{8,-50},{8,-8.88889}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CabinWindow;
