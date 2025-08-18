within DynTherM.Tests.Distributed;
model RectangularCV
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.EGW(X=0.1)
    "Refrigerant";

  parameter Modelica.Units.SI.Length L "Length of the control volume";
  parameter Modelica.Units.SI.Length W "Width of the control volume";
  parameter Modelica.Units.SI.Length H "Height of the control volume";
  parameter Modelica.Units.SI.Length t "Solid wall thickness";
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

  BoundaryConditions.ZeroDimensional.flow_source ECSFlow(
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
  BoundaryConditions.ZeroDimensional.pressure_sink pressureSink(redeclare
      package Medium = Refrigerant)
    annotation (Placement(transformation(extent={{68,-12},{92,12}})));
  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.steadyState) annotation (Placement(transformation(extent={{60,60},{100,100}})));
  Components.OneDimensional.RectangularFluxCV cv(
    redeclare package Medium = Refrigerant,
    L=L,
    W=W,
    H=H,
    t_north=t,
    t_east=t,
    t_south=t,
    t_west=t,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start)
    annotation (Placement(transformation(extent={{-30,-30},{30,30}})));

  BoundaryConditions.ZeroDimensional.thermal_flux thermal(phi=phi)
    annotation (Placement(transformation(extent={{-16,52},{8,68}})));
equation

  connect(m_fromMix,ECSFlow. in_massFlow)
    annotation (Line(points={{-110,20},{-91.2,20},{-91.2,9.8}},
                                                           color={0,0,127}));
  connect(T_fromMix,ECSFlow. in_T)
    annotation (Line(points={{-110,50},{-82.8,50},{-82.8,9.8}},
                                                           color={0,0,127}));
  connect(ECSFlow.outlet, cv.inlet) annotation (Line(points={{-66,-3.44169e-15},
          {-45,-3.44169e-15},{-45,0},{-24,0}}, color={0,0,0}));
  connect(cv.outlet, pressureSink.inlet)
    annotation (Line(points={{24,0},{68,0}}, color={0,0,0}));
  connect(thermal.thermal_flux, cv.solid_surface_east) annotation (Line(points={
          {0,60},{0,34},{-6,34},{-6,21}}, color={255,127,0}));
  connect(thermal.thermal_flux, cv.solid_surface_south)
    annotation (Line(points={{0,60},{0,34},{6,34},{6,21}}, color={255,127,0}));
  connect(thermal.thermal_flux, cv.solid_surface_west) annotation (Line(points={
          {0,60},{0,34},{17.4,34},{17.4,21}}, color={255,127,0}));
  connect(thermal.thermal_flux, cv.solid_surface_north) annotation (Line(points=
         {{0,60},{0,34},{-18,34},{-18,21}}, color={255,127,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RectangularCV;
