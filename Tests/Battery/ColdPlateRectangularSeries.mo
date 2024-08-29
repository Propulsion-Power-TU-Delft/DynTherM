within DynTherM.Tests.Battery;
model ColdPlateRectangularSeries
    package Coolant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1);

    parameter Integer N_cv=1;

    // Geometry
    parameter Length L=0.35;
    parameter Length W_channel=0.02;
    parameter Length H_plate=2e-3;
    parameter Length H_channel=0.8e-3;
    parameter Length d=0.01;

    parameter Temperature T_fluid=298.15;
    parameter MassFlowRate m_flow=0.04416;

    // Initialization
    parameter Temperature T_start_cell=298.15;
    parameter Temperature T_start_solid=298.15;
    parameter Temperature T_start_fluid=298.15;
    parameter Pressure P_start=3e5;
    parameter MassFlowRate m_flow_start=0.04416;
    parameter Velocity u_start=3;
    parameter Density rho_start=1e3;
    parameter Pressure dP_start=1e4;
    parameter ReynoldsNumber Re_start=5e3;
    parameter PrandtlNumber Pr_start=7;

    DynTherM.BoundaryConditions.flow_source flow_source(
      redeclare package Medium = Coolant,
    P_nom=400000,
      T_nom=T_fluid,
      massFlow_nom=m_flow,
      allowFlowReversal=environment.allowFlowReversal,
      use_in_massFlow=false,
      use_in_T=false,
      P_start=P_start,
      T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{-124,112},{-100,88}})));
    DynTherM.BoundaryConditions.pressure_sink pressure_sink(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      use_ambient=false,
    P_di=150000,
    T_di=T_fluid,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{-100,50},{-120,70}})));
    inner DynTherM.Components.Environment environment(allowFlowReversal=false,
      initOpt=DynTherM.Choices.InitOpt.fixedState)
      annotation (Placement(transformation(extent={{78,78},{116,116}})));
    DynTherM.Components.OneDimensional.RectangularChannel1D channel1(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      initOpt=environment.initOpt,
    use_inertia=false,
    L=L,
      W=W_channel,
      H=H_channel,
      t_north=(H_plate - H_channel)/2,
      t_east=d/2,
      t_south=(H_plate - H_channel)/2,
      t_west=d/2,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      m_flow_start=m_flow_start,
    m_flow_mc_start=m_flow_start,
      rho_start=rho_start,
      u_start=u_start,
      dP_start=dP_start,
      Re_start=Re_start,
      Pr_start=Pr_start,
      N_cv=N_cv,
      N_channels=1)
      annotation (Placement(transformation(extent={{-80,80},{-40,120}})));
    DynTherM.Components.OneDimensional.RectangularChannel1D channel6(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      initOpt=environment.initOpt,
    use_inertia=false,
    L=L,
      W=W_channel,
      H=H_channel,
      t_north=(H_plate - H_channel)/2,
      t_east=d/2,
      t_south=(H_plate - H_channel)/2,
      t_west=d/2,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      m_flow_start=m_flow_start,
    m_flow_mc_start=m_flow_start,
      rho_start=rho_start,
      u_start=u_start,
      dP_start=dP_start,
      Re_start=Re_start,
      Pr_start=Pr_start,
      N_cv=N_cv,
      N_channels=1)
      annotation (Placement(transformation(extent={{-40,40},{-80,80}})));
    DynTherM.Components.OneDimensional.RectangularChannel1D channel5(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      initOpt=environment.initOpt,
    use_inertia=false,
    L=L,
      W=W_channel,
      H=H_channel,
      t_north=(H_plate - H_channel)/2,
      t_east=d/2,
      t_south=(H_plate - H_channel)/2,
      t_west=d/2,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      m_flow_start=m_flow_start,
    m_flow_mc_start=m_flow_start,
      rho_start=rho_start,
      u_start=u_start,
      dP_start=dP_start,
      Re_start=Re_start,
      Pr_start=Pr_start,
      N_cv=N_cv,
      N_channels=1)
      annotation (Placement(transformation(extent={{-80,0},{-40,40}})));
    DynTherM.Components.OneDimensional.RectangularChannel1D channel4(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      initOpt=environment.initOpt,
    use_inertia=false,
    L=L,
      W=W_channel,
      H=H_channel,
      t_north=(H_plate - H_channel)/2,
      t_east=d/2,
      t_south=(H_plate - H_channel)/2,
      t_west=d/2,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      m_flow_start=m_flow_start,
    m_flow_mc_start=m_flow_start,
      rho_start=rho_start,
      u_start=u_start,
      dP_start=dP_start,
      Re_start=Re_start,
      Pr_start=Pr_start,
      N_cv=N_cv,
      N_channels=1)
      annotation (Placement(transformation(extent={{-40,-40},{-80,0}})));
    DynTherM.Components.OneDimensional.RectangularChannel1D channel3(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      initOpt=environment.initOpt,
    use_inertia=false,
    L=L,
      W=W_channel,
      H=H_channel,
      t_north=(H_plate - H_channel)/2,
      t_east=d/2,
      t_south=(H_plate - H_channel)/2,
      t_west=d/2,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      m_flow_start=m_flow_start,
    m_flow_mc_start=m_flow_start,
      rho_start=rho_start,
      u_start=u_start,
      dP_start=dP_start,
      Re_start=Re_start,
      Pr_start=Pr_start,
      N_cv=N_cv,
      N_channels=1)
      annotation (Placement(transformation(extent={{-80,-80},{-40,-40}})));
    DynTherM.Components.OneDimensional.RectangularChannel1D channel2(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      initOpt=environment.initOpt,
    use_inertia=false,
    L=L,
      W=W_channel,
      H=H_channel,
      t_north=(H_plate - H_channel)/2,
      t_east=d/2,
      t_south=(H_plate - H_channel)/2,
      t_west=d/2,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      m_flow_start=m_flow_start,
    m_flow_mc_start=m_flow_start,
      rho_start=rho_start,
      u_start=u_start,
      dP_start=dP_start,
      Re_start=Re_start,
      Pr_start=Pr_start,
      N_cv=N_cv,
      N_channels=1)
      annotation (Placement(transformation(extent={{-40,-120},{-80,-80}})));
  DynTherM.Systems.Battery.ColdPlateRectangularSeries cold_plate(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    initOpt=environment.initOpt,
    use_inertia=true,
    L=L,
    W=W_channel,
    H=H_channel,
    t_vertical=(H_plate - H_channel)/2,
    t_horizontal=d/2,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    m_flow_start=m_flow_start,
    u_start=u_start,
    rho_start=rho_start,
    dP_start=dP_start,
    Re_start=Re_start,
    Pr_start=Pr_start,
    N_channels=6,
    N_cv=N_cv)
    annotation (Placement(transformation(extent={{40,-44},{120,50}})));
    DynTherM.BoundaryConditions.flow_source flow_source1(
    redeclare package Medium = Coolant,
    P_nom=400000,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{8,72},{32,48}})));
    DynTherM.BoundaryConditions.pressure_sink pressure_sink1(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=150000,
    T_di=T_fluid,
    P_start=P_start,
    T_start=T_start_fluid)
      annotation (Placement(transformation(extent={{30,-50},{10,-30}})));
  DynTherM.BoundaryConditions.thermal_distributed BC(
    Nx=cold_plate.N_channels,
    Ny=cold_plate.N_cv,
    use_di_Q=false,
    use_in_Q=true)
    annotation (Placement(transformation(extent={{106,-26},{54,-54}})));
  Modelica.Blocks.Sources.Step step(
    height=-20*12,
    offset=80*12,
    startTime=500)
    annotation (Placement(transformation(extent={{120,-100},{100,-80}})));
  DynTherM.BoundaryConditions.thermal_distributed BC1(
    Nx=N_cv,
    Ny=1,
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-6.5,-6.5},{6.5,6.5}},
        rotation=0,
        origin={-75.5,114.5}), iconTransformation(extent={{-30,-98},{30,-38}})));
  DynTherM.BoundaryConditions.thermal_distributed BC6(
    Nx=N_cv,
    Ny=1,
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-6.5,-6.5},{6.5,6.5}},
        rotation=0,
        origin={-45.5,74.5}), iconTransformation(extent={{-30,-98},{30,-38}})));
  DynTherM.BoundaryConditions.thermal_distributed BC5(
    Nx=N_cv,
    Ny=1,
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-6.5,-6.5},{6.5,6.5}},
        rotation=0,
        origin={-75.5,34.5}), iconTransformation(extent={{-30,-98},{30,-38}})));
  DynTherM.BoundaryConditions.thermal_distributed BC4(
    Nx=N_cv,
    Ny=1,
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-6.5,-6.5},{6.5,6.5}},
        rotation=0,
        origin={-45.5,-5.5}), iconTransformation(extent={{-30,-98},{30,-38}})));
  DynTherM.BoundaryConditions.thermal_distributed BC3(
    Nx=N_cv,
    Ny=1,
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-6.5,-6.5},{6.5,6.5}},
        rotation=0,
        origin={-75.5,-45.5}), iconTransformation(extent={{-30,-98},{30,-38}})));
  DynTherM.BoundaryConditions.thermal_distributed BC2(
    Nx=N_cv,
    Ny=1,
    use_di_Q=false,
    use_in_Q=true) annotation (Placement(transformation(
        extent={{-6.5,-6.5},{6.5,6.5}},
        rotation=0,
        origin={-45.5,-85.5}), iconTransformation(extent={{-30,-98},{30,-38}})));
  Modelica.Blocks.Math.Gain gain(k=1/(6*N_cv))
    annotation (Placement(transformation(extent={{60,-100},{40,-80}})));
equation
    connect(pressure_sink.inlet,channel6. outlet)
      annotation (Line(points={{-100,60},{-80,60}}, color={0,0,0}));
    connect(flow_source.outlet, channel1.inlet)
      annotation (Line(points={{-100,100},{-80,100}},color={0,0,0}));
    connect(channel4.solid_surface_west,channel3. solid_surface_west) annotation (
       Line(points={{-75,-10.8},{-75,-36},{-45,-36},{-45,-50.8}},
                                                                color={191,0,0}));
    connect(channel5.solid_surface_east, channel4.solid_surface_east)
      annotation (Line(points={{-65,29.2},{-65,4},{-55,4},{-55,-10.8}},
                                                             color={191,0,0}));
    connect(channel6.solid_surface_west, channel5.solid_surface_west) annotation (
       Line(points={{-75,69.2},{-75,44},{-45,44},{-45,29.2}},
                                                        color={191,0,0}));
    connect(channel3.solid_surface_east, channel2.solid_surface_east) annotation (
       Line(points={{-65,-50.8},{-65,-76},{-55,-76},{-55,-90.8}},
                                                        color={191,0,0}));
    connect(channel1.solid_surface_east,channel6. solid_surface_east) annotation (
       Line(points={{-65,109.2},{-65,84},{-55,84},{-55,69.2}},
                                                         color={191,0,0}));
    connect(channel1.outlet, channel2.inlet) annotation (Line(points={{-40,100},
          {0,100},{0,-100},{-40,-100}},color={0,0,0}));
    connect(channel3.outlet, channel4.inlet) annotation (Line(points={{-40,-60},
          {-20,-60},{-20,-20},{-40,-20}},
                                     color={0,0,0}));
    connect(channel5.outlet, channel6.inlet)
      annotation (Line(points={{-40,20},{-20,20},{-20,60},{-40,60}},
                                                                 color={0,0,0}));
    connect(channel4.outlet, channel5.inlet) annotation (Line(points={{-80,-20},
          {-100,-20},{-100,20},{-80,20}},
                                     color={0,0,0}));
    connect(channel2.outlet, channel3.inlet) annotation (Line(points={{-80,-100},
          {-100,-100},{-100,-60},{-80,-60}},color={0,0,0}));
  connect(flow_source1.outlet, cold_plate.inlet)
    annotation (Line(points={{32,60},{40,60},{40,23.3667}}, color={0,0,0}));
  connect(cold_plate.outlet, pressure_sink1.inlet) annotation (Line(points={{40,
          13.9667},{30,13.9667},{30,-40}}, color={0,0,0}));
  connect(BC.thermal, cold_plate.upper_surface)
    annotation (Line(points={{80,-40},{80,-23.6333}}, color={191,0,0}));
  connect(step.y, BC.in_Q) annotation (Line(points={{99,-90},{88.6667,-90},{
          88.6667,-45.6}}, color={0,0,127}));
  connect(BC2.thermal, channel2.solid_surface_north) annotation (Line(points={{
          -45.5,-85.5},{-46,-85.5},{-46,-90.8},{-45,-90.8}}, color={191,0,0}));
  connect(BC3.thermal, channel3.solid_surface_north) annotation (Line(points={{
          -75.5,-45.5},{-76,-45.5},{-76,-50.8},{-75,-50.8}}, color={191,0,0}));
  connect(BC4.thermal, channel4.solid_surface_north) annotation (Line(points={{
          -45.5,-5.5},{-46,-5.5},{-46,-10.8},{-45,-10.8}}, color={191,0,0}));
  connect(BC5.thermal, channel5.solid_surface_north) annotation (Line(points={{
          -75.5,34.5},{-76,34.5},{-76,29.2},{-75,29.2}}, color={191,0,0}));
  connect(BC6.thermal, channel6.solid_surface_north) annotation (Line(points={{
          -45.5,74.5},{-46,74.5},{-46,69.2},{-45,69.2}}, color={191,0,0}));
  connect(BC1.thermal, channel1.solid_surface_north) annotation (Line(points={{
          -75.5,114.5},{-76,114.5},{-76,109.2},{-75,109.2}}, color={191,0,0}));
  connect(step.y, gain.u)
    annotation (Line(points={{99,-90},{62,-90}}, color={0,0,127}));
  connect(gain.y, BC2.in_Q) annotation (Line(points={{39,-90},{-10,-90},{-10,
          -82.9},{-47.6667,-82.9}}, color={0,0,127}));
  connect(gain.y, BC3.in_Q) annotation (Line(points={{39,-90},{-10,-90},{-10,
          -42.9},{-77.6667,-42.9}}, color={0,0,127}));
  connect(gain.y, BC4.in_Q) annotation (Line(points={{39,-90},{-10,-90},{-10,
          -2.9},{-47.6667,-2.9}}, color={0,0,127}));
  connect(gain.y, BC5.in_Q) annotation (Line(points={{39,-90},{-10,-90},{-10,
          37.1},{-77.6667,37.1}}, color={0,0,127}));
  connect(gain.y, BC6.in_Q) annotation (Line(points={{39,-90},{-10,-90},{-10,
          77.1},{-47.6667,77.1}}, color={0,0,127}));
  connect(gain.y, BC1.in_Q) annotation (Line(points={{39,-90},{-10,-90},{-10,
          117.1},{-77.6667,117.1}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-120},
            {140,120}})),                                          Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-140,-120},{140,120}})),
      experiment(
        StopTime=1200,
        Interval=1,
        __Dymola_Algorithm="Dassl"));
end ColdPlateRectangularSeries;
