within DynTherM.Tests.Distributed;
model RectangularParallel1D
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.MIL_PRF_23699;  //Modelica.Media.Water.StandardWater "Refrigerant";

  parameter Integer N(min=1) "Number of longitudinal sections in which the tube is discretized";
  parameter Integer N_channels(min=1) "Number of channels in parallel";

  parameter Modelica.Units.SI.Length L "Channel length";
  parameter Modelica.Units.SI.Length W "Width of the control volume";
  parameter Modelica.Units.SI.Length H "Height of the control volume";
  parameter Modelica.Units.SI.Length t_north "Thickness of north wall" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length t_east "Thickness of east wall" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length t_south "Thickness of south wall" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length t_west "Thickness of west wall" annotation (Dialog(tab="Geometry"));

  parameter Modelica.Units.SI.HeatFlux phi;
  parameter Modelica.Units.SI.Temperature T_start_solid
    "Temperature of solid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Temperature T_start_fluid
    "Temperature of fluid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Pressure P_start=101325
    "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.MassFraction X_start[Refrigerant.nX]=Refrigerant.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Refrigerant.ThermodynamicState state_start = Refrigerant.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));

  BoundaryConditions.flow_source ECSFlow(
    redeclare package Medium = Refrigerant,
    use_in_massFlow=true,
    use_in_T=true) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-80,0})));
  Modelica.Blocks.Interfaces.RealInput T_fromMix annotation (Placement(
        transformation(extent={{-130,30},{-90,70}}), iconTransformation(
          extent={{-106,-30},{-86,-10}})));
  Modelica.Blocks.Interfaces.RealInput m_fromMix annotation (Placement(
        transformation(extent={{-130,0},{-90,40}}),  iconTransformation(
          extent={{-106,-70},{-86,-50}})));
  BoundaryConditions.pressure_sink pressureSink(redeclare package Medium =
        Refrigerant)
    annotation (Placement(transformation(extent={{68,-12},{92,12}})));
  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.steadyState) annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.OneDimensional.RectangularChannels1D channel1D(
    redeclare package Medium = Refrigerant,
    L=L,
    H=H,
    W=W,
    t_north=t_north,
    t_south=t_south,
    t_east=t_east,
    t_west=t_west,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    N=N,
    N_channels=N_channels)
         annotation (Placement(transformation(extent={{-30,-30},{30,30}})));

  BoundaryConditions.thermal_flux_distributed thermal_distributed(
    Nx=N,
    Ny=1,
    phi=phi*ones(N, 1))
    annotation (Placement(transformation(extent={{-40,48},{-4,70}})));
  Modelica.Blocks.Sources.Constant const2(k=0)
    annotation (Placement(transformation(extent={{20,80},{40,100}})));
  Modelica.Blocks.Sources.Constant const1(k=0)
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
equation

  connect(m_fromMix,ECSFlow. in_massFlow)
    annotation (Line(points={{-110,20},{-91.2,20},{-91.2,9.8}},
                                                           color={0,0,127}));
  connect(T_fromMix,ECSFlow. in_T)
    annotation (Line(points={{-110,50},{-82.8,50},{-82.8,9.8}},
                                                           color={0,0,127}));
  connect(ECSFlow.outlet, channel1D.inlet) annotation (Line(points={{-66,-3.44169e-15},
          {-48,-3.44169e-15},{-48,0},{-30,0}}, color={0,0,0}));
  connect(channel1D.outlet, pressureSink.inlet)
    annotation (Line(points={{30,0},{68,0}}, color={0,0,0}));
  connect(thermal_distributed.thermal_flux, channel1D.solid_surface_north)
    annotation (Line(points={{-22,59},{-22,13.8},{-22.5,13.8}}, color={255,
          127,0}));
  connect(const2.y, environment.altitude) annotation (Line(points={{41,90},{48,90},
          {48,72},{60,72}}, color={0,0,127}));
  connect(const1.y, environment.V_inf) annotation (Line(points={{41,50},{48,50},
          {48,64},{60,64}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RectangularParallel1D;
