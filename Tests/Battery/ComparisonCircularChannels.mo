within DynTherM.Tests.Battery;
model ComparisonCircularChannels
  "This test compares the performance of circular channels which are modelled with different approaches"

 package Coolant = DynTherM.Media.IncompressibleTableBased.MEG_Polestar;
// package Coolant = Modelica.Media.Water.StandardWater;
// package Coolant = Modelica.Media.Water.ConstantPropertyLiquidWater;

    parameter Temperature T_fluid=298.15;
    parameter MassFlowRate m_flow=0.04416;

    // Geometery
    parameter Length L = 0.35 "Lenght of the channel";
    parameter Length t = 0.006 "Thickness of the cold plate";
    parameter Length d = 0.007 "Center to center distance between the parallel channels";
    parameter Length R_int = 0.0025 "Internal radius of the Channel";

    // Initialization
    parameter Temperature T_start_solid=298.15;
    parameter Temperature T_start_fluid=298.15;
    parameter Pressure P_start=1e5;
    parameter MassFlowRate m_flow_start=0.04416;
    parameter Velocity u_start=2;
    parameter Density rho_start=1e3;
    parameter Pressure dP_start=2e3;
    parameter ReynoldsNumber Re_start=3e3;
    parameter PrandtlNumber Pr_start=30;

    // Discretization
    parameter Integer N_cv= 5 "Number of control volumes in which the cooling channels are discretized";

    BoundaryConditions.pressure_sink          pressure_sink(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=100000,
    T_di=T_fluid,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{38,44},{56,62}})));
    BoundaryConditions.flow_source          flow_source(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{-90,64},{-70,44}})));
  Components.TwoDimensional.ColdPlateCircularChannel1D Channel2D(
    redeclare package Medium = Coolant,
    allowFlowReversal=true,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    L=L,
    t(displayUnit="mm") = t,
    d(displayUnit="mm") = d,
    R_int(displayUnit="mm") = R_int,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    m_flow_start=m_flow_start,
    rho_start=rho_start,
    u_start=u_start,
    dP_start=dP_start,
    Re_start=Re_start,
    Pr_start=Pr_start,
    N_cv=N_cv,
    Nt=3,
    N_channels=1)
    annotation (Placement(transformation(extent={{-50,22},{22,86}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowChannel2D(Nx=N_cv, Ny=1)
    annotation (Placement(transformation(
        extent={{-12,-6},{12,6}},
        rotation=90,
        origin={16,20})));

  Components.OneDimensional.CircularAsymmetricChannel1D Asymmetric(
    redeclare package Medium = Coolant,
    allowFlowReversal=true,
    L=L,
    R_ext_north(displayUnit="mm") = t/2,
    R_ext_east(displayUnit="mm") = d/2,
    R_ext_south(displayUnit="mm") = t/2,
    R_ext_west(displayUnit="mm") = d/2,
    R_int(displayUnit="mm") = R_int,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    m_flow_start=m_flow_start,
    rho_start=rho_start,
    u_start=u_start,
    dP_start=dP_start,
    Re_start=Re_start,
    Pr_start=Pr_start,
    N_cv=N_cv,
    N_channels=1)
    annotation (Placement(transformation(extent={{-48,-82},{24,-10}})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowAsymmetric(Nx=N_cv, Ny=1)
    annotation (Placement(transformation(
        extent={{-12,-6},{12,6}},
        rotation=90,
        origin={16,-10})));
    BoundaryConditions.flow_source          flow_source1(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{-88,-36},{-70,-54}})));
    BoundaryConditions.pressure_sink          pressure_sink1(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=100000,
    T_di=T_fluid,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{42,-52},{58,-36}})));
  BoundaryConditions.thermal FixedTemperature(
    T=313.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{70,-52},{90,60}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heatSensorChannel2D
    annotation (Placement(transformation(extent={{34,10},{54,30}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heatSensorAsymmetric
    annotation (Placement(transformation(extent={{34,-20},{54,0}})));
equation
  connect(flow_source.outlet, Channel2D.inlet)
    annotation (Line(points={{-70,54},{-50,54}}, color={0,0,0}));
  connect(heatFlowChannel2D.distributed, Channel2D.BottomSurface) annotation (
      Line(points={{12.4,20},{3.28,20},{3.28,38.96}}, color={191,0,0}));
  connect(Channel2D.outlet, pressure_sink.inlet)
    annotation (Line(points={{22,54},{32,54},{32,53},{38,53}}, color={0,0,0}));
  connect(Asymmetric.solid_surface_north, heatFlowAsymmetric.distributed)
    annotation (Line(points={{-39,-29.44},{-38.5,-29.44},{-38.5,-10},{12.4,-10}},
        color={191,0,0}));
  connect(flow_source1.outlet, Asymmetric.inlet) annotation (Line(points={{-70,-45},
          {-62,-45},{-62,-46},{-48,-46}}, color={0,0,0}));
  connect(Asymmetric.outlet, pressure_sink1.inlet) annotation (Line(points={{24,
          -46},{34,-46},{34,-44},{42,-44}}, color={0,0,0}));
  connect(heatFlowChannel2D.single, heatSensorChannel2D.port_a)
    annotation (Line(points={{19.6,20},{34,20}}, color={191,0,0}));
  connect(heatSensorAsymmetric.port_b, FixedTemperature.thermal) annotation (
      Line(points={{54,-10},{82,-10},{82,4},{83.3333,4}}, color={191,0,0}));
  connect(heatFlowAsymmetric.single, heatSensorAsymmetric.port_a)
    annotation (Line(points={{19.6,-10},{34,-10}}, color={191,0,0}));
  connect(heatSensorChannel2D.port_b, FixedTemperature.thermal) annotation (
      Line(points={{54,20},{84,20},{84,4},{83.3333,4}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Polygon(
          origin={24,16},
          lineColor={78,138,73},
          fillColor={78,138,73},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=1200, __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>This test compares the fluid flow heat transfer in a circular channel that is modelled with 2 different strategies.</p>
<ul>
<li>Channel model &quot;<i>Asymmetric</i>&quot; assumes that the heat transfer flows in radial direction. As a result there is a little over approximation of surface area on the top and bottom surface of the channels which in reality are flat surfaces.</li>
<li>Channel mode &quot;<i>Channel2D</i>&quot; uses a combination of rectangular and plano-concave surfaces to model the heat transfer in two direction. </li>
</ul>
<h4>Results</h4>
<p>The results of this test shows that at the coolant mass flow rate of 0.04416kg/s and inlet temperature of 25<sup><span style=\"font-size: 6pt;\">0</span></sup>C, if the outer surface tempeature of the channel is kept 40<sup><span style=\"font-size: 6pt;\">0</span></sup>C, the heat transfer rate for &quot;<i>Asymmertic</i>&quot; model is 320W while for the &quot;<i>Channel2D</i>&quot; model, it is 301W. &quot;<i>Asymmertic</i>&quot; model overstates the heat transfer rate. This is mainly becuase of the inherent differences in the way the channels are modelled. </p>
</html>"));
end ComparisonCircularChannels;
