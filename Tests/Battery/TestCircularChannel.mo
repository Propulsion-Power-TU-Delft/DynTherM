within DynTherM.Tests.Battery;
model TestCircularChannel
  Components.OneDimensional.ColdPlateCircularChannel1D Channel1(
    redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
    L=1,
    t=0.3,
    d=0.3,
    R_int=0.05,
    N_cv=10,
    N_channels=1)
    annotation (Placement(transformation(extent={{-30,-36},{16,12}})));

  BoundaryConditions.pressure_sink pressure_sink(
    redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false,
    P_di=100000,
    T_di=298.15)
    annotation (Placement(transformation(extent={{-74,-90},{-94,-70}})));

  BoundaryConditions.flow_source flow_source(
    redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
    P_nom=100000,
    T_nom=298.15,
    massFlow_nom=0.04,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-96,-56},{-72,-32}})));

  Systems.Battery.PouchModuleFirewall battery_module(
    W_cell=0.35,
    H_cell=0.1,
    t_cell(displayUnit="mm") = 0.01,
    t_fw(displayUnit="mm") = 0.001,
    C_nom(displayUnit="Ah") = 230400,
    initOpt=environment.initOpt,
    SoC_start=0.1,
    Tstart=298.15,
    N_cv=12,
    N_parallel=1,
    N_series=10)
    annotation (Placement(transformation(extent={{6,4},{86,84}})));
  inner Components.Environment environment(allowFlowReversal=false, initOpt=
        DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-102,56},{-58,100}})));
  Modelica.Blocks.Sources.TimeTable I_charging(table=[0,200; 200,200; 200,200;
        500,200; 501,120; 700,120; 701,100; 900,100; 901,120; 1000,120; 1001,70;
        1200,70])
             annotation (Placement(transformation(extent={{102,74},{86,90}})));
  Components.OneDimensional.ColdPlateCircularChannel1D channel2(
    redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
    L=1,
    t=0.3,
    d=0.3,
    R_int=0.05,
    N_cv=10,
    N_channels=1)
    annotation (Placement(transformation(extent={{22,-62},{-24,-110}})));

  CustomInterfaces.Adaptors.InvertDistributedHeatflows
    invertDistributedHeatflows(Nx=10, Ny=3)
    annotation (Placement(transformation(extent={{0,-66},{20,-46}})));
  CustomInterfaces.Adaptors.InvertDistributedHeatflows
    invertDistributedHeatflows1(Nx=10, Ny=1)
    annotation (Placement(transformation(extent={{48,-36},{68,-16}})));
equation
  connect(flow_source.outlet, Channel1.inlet) annotation (Line(points={{-72,-44},
          {-38,-44},{-38,-12},{-30,-12}}, color={0,0,0}));
  connect(battery_module.Bottom, Channel1.TopSurface) annotation (Line(points={{38,17.6},
          {38,-0.72},{4.04,-0.72}},          color={191,0,0}));
  connect(battery_module.I, I_charging.y) annotation (Line(points={{42.8,63.2},
          {42.8,92},{82,92},{82,94},{85.2,94},{85.2,82}}, color={0,0,127}));
  connect(pressure_sink.inlet, channel2.outlet) annotation (Line(points={{-74,-80},
          {-34,-80},{-34,-86},{-24,-86}}, color={0,0,0}));
  connect(Channel1.outlet, channel2.inlet) annotation (Line(points={{16,-12},{
          32,-12},{32,-86},{22,-86}},
                                   color={0,0,0}));


  connect(channel2.EastSide, invertDistributedHeatflows.distributedHeatPort_out)
    annotation (Line(points={{9.81,-74.72},{10.1,-74.72},{10.1,-62.9}}, color={
          191,0,0}));
  connect(invertDistributedHeatflows.distributedHeatPort_in, Channel1.EastSide)
    annotation (Line(points={{10.1,-49.1},{-6,-49.1},{-6,-50},{-20,-50},{-20,
          -23.28},{-17.81,-23.28}}, color={191,0,0}));
  connect(battery_module.Bottom, invertDistributedHeatflows1.distributedHeatPort_in)
    annotation (Line(points={{38,17.6},{38,-10},{58.1,-10},{58.1,-19.1}}, color
        ={191,0,0}));
  connect(invertDistributedHeatflows1.distributedHeatPort_out, channel2.BottomSurface)
    annotation (Line(points={{58.1,-32.9},{58.1,-40},{-22,-40},{-22,-58},{
          -12.04,-58},{-12.04,-74.72}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end TestCircularChannel;
