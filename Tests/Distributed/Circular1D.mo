within DynTherM.Tests.Distributed;
model Circular1D
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1)
    "Refrigerant";

  parameter Integer N=3 "Number of longitudinal sections in which the tube is discretized";
  parameter Modelica.Units.SI.Length L "Tube length";
  parameter Modelica.Units.SI.Length R_ext "Tube external radius";
  parameter Modelica.Units.SI.Length R_int "Tube internal radius";
  parameter Modelica.Units.SI.HeatFlowRate Q_flow;
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
  Components.OneDimensional.CircularChannel1D channel1D(
    redeclare package Medium = Refrigerant,
    L=L,
    R_ext=R_ext,
    R_int=R_int,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    N=N) annotation (Placement(transformation(extent={{-30,-30},{30,30}})));

  BoundaryConditions.thermal_distributed thermal_distributed(N=N, Q=Q_flow/N*
        ones(N))
    annotation (Placement(transformation(extent={{-18,42},{18,64}})));
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
  connect(thermal_distributed.thermal, channel1D.solid_surface) annotation (
      Line(points={{-3.55271e-15,53},{-3.55271e-15,33.55},{0,33.55},{0,14.1}},
        color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Circular1D;
