within DynTherM.Tests;
package Distributed

  model test_circularCV
    CircularCV OneD(
      L(displayUnit="mm") = 0.4826,
      R_ext(displayUnit="mm") = 0.003,
      R_int(displayUnit="mm") = 0.0025,
      Q_flow(displayUnit="kW") = -10000,
      T_start_solid=323.15,
      T_start_fluid=323.15)
      annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
    Modelica.Blocks.Sources.Constant m(k=0.3)
      annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
    Modelica.Blocks.Sources.Constant T(k=273.15 + 50)
      annotation (Placement(transformation(extent={{-80,-14},{-60,6}})));
  equation
    connect(T.y, OneD.T_fromMix)
      annotation (Line(points={{-59,-4},{-19.2,-4}}, color={0,0,127}));
    connect(m.y, OneD.m_fromMix) annotation (Line(points={{-59,-50},{-40,-50},{
            -40,-12},{-19.2,-12}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end test_circularCV;

  model test_rectangularCV
    RectangularCV OneD(
      L(displayUnit="mm") = 0.4826,
      W(displayUnit="mm") = 0.0559,
      H(displayUnit="mm") = 0.00196,
      t(displayUnit="mm") = 0.001,
      phi=-13800,
      T_start_solid=323.15,
      T_start_fluid=323.15)
      annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
    Modelica.Blocks.Sources.Constant m(k=0.3)
      annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
    Modelica.Blocks.Sources.Constant T(k=273.15 + 50)
      annotation (Placement(transformation(extent={{-80,-14},{-60,6}})));
  equation
    connect(T.y, OneD.T_fromMix)
      annotation (Line(points={{-59,-4},{-19.2,-4}}, color={0,0,127}));
    connect(m.y, OneD.m_fromMix) annotation (Line(points={{-59,-50},{-40,-50},{
            -40,-12},{-19.2,-12}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end test_rectangularCV;

  model test_circular1D
    Circular1D OneD(
      L(displayUnit="mm") = 0.4826,
      R_ext(displayUnit="mm") = 0.003,
      R_int(displayUnit="mm") = 0.0025,
      Q_flow(displayUnit="kW") = -10000,
      T_start_solid=323.15,
      T_start_fluid=323.15)
      annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
    Modelica.Blocks.Sources.Constant m(k=0.3)
      annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
    Modelica.Blocks.Sources.Constant T(k=273.15 + 50)
      annotation (Placement(transformation(extent={{-80,-14},{-60,6}})));
  equation
    connect(T.y, OneD.T_fromMix)
      annotation (Line(points={{-59,-4},{-19.2,-4}}, color={0,0,127}));
    connect(m.y, OneD.m_fromMix) annotation (Line(points={{-59,-50},{-40,-50},{
            -40,-12},{-19.2,-12}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end test_circular1D;

  model test_rectangular1D
    Rectangular1D OneD(
      L(displayUnit="mm") = 0.4826,
      W(displayUnit="mm") = 0.0559,
      H(displayUnit="mm") = 0.00196,
      t_north(displayUnit="mm") = 0.001,
      t_east(displayUnit="mm") = 0.0005,
      t_south(displayUnit="mm") = 0.001,
      t_west(displayUnit="mm") = 0.0005,
      phi=-13800,
      T_start_solid=323.15,
      T_start_fluid=323.15)
      annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
    Modelica.Blocks.Sources.Constant m(k=0.3)
      annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
    Modelica.Blocks.Sources.Constant T(k=273.15 + 50)
      annotation (Placement(transformation(extent={{-80,-14},{-60,6}})));
  equation
    connect(T.y, OneD.T_fromMix)
      annotation (Line(points={{-59,-4},{-19.2,-4}}, color={0,0,127}));
    connect(m.y, OneD.m_fromMix) annotation (Line(points={{-59,-50},{-40,-50},{
            -40,-12},{-19.2,-12}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end test_rectangular1D;

  model test_rectangular2D
    Rectangular2D TwoD(
      N_channels=3,
      L(displayUnit="mm") = 0.4826,
      W(displayUnit="mm") = 0.0559,
      H(displayUnit="mm") = 0.00196,
      t_ext(displayUnit="mm") = 0.001,
      t_int(displayUnit="mm") = 0.0005,
      m_flow=0.3,
      T_in=323.15,
      phi=-13800,
      T_start_solid=323.15,
      T_start_fluid=323.15)
      annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end test_rectangular2D;

  model test_rectangular2D_manual
    Rectangular2D_manual TwoD(
      L(displayUnit="mm") = 0.4826,
      W(displayUnit="mm") = 0.0559,
      H(displayUnit="mm") = 0.00196,
      t_ext(displayUnit="mm") = 0.001,
      t_int(displayUnit="mm") = 0.0005,
      phi=-13800,
      T_start_solid=323.15,
      T_start_fluid=323.15)
      annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
    Modelica.Blocks.Sources.Constant m(k=0.1)
      annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
    Modelica.Blocks.Sources.Constant T(k=273.15 + 50)
      annotation (Placement(transformation(extent={{-80,-14},{-60,6}})));
  equation
    connect(T.y, TwoD.T)
      annotation (Line(points={{-59,-4},{-19.2,-4}}, color={0,0,127}));
    connect(m.y, TwoD.m) annotation (Line(points={{-59,-50},{-40,-50},{-40,-12},
            {-19.2,-12}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end test_rectangular2D_manual;

  model CircularCV
    package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1)
      "Refrigerant";

    parameter Modelica.Units.SI.Length L "Length of the control volume";
    parameter Modelica.Units.SI.Length R_ext "External radius of the control volume";
    parameter Modelica.Units.SI.Length R_int "Internal radius of the control volume";
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
    Components.OneDimensional.CircularCV cv(
      redeclare package Medium = Refrigerant,
      L=L,
      R_ext=R_ext,
      R_int=R_int,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      X_start=X_start,
      state_start=state_start,
      m_flow_start=m_flow_start)
      annotation (Placement(transformation(extent={{-30,-30},{30,30}})));

    BoundaryConditions.thermal thermal(Q=Q_flow)
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
    connect(thermal.thermal, cv.solid_surface)
      annotation (Line(points={{0,60},{0,24}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end CircularCV;

  model RectangularCV
    package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1)
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
    Components.OneDimensional.RectangularCV cv(
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

    BoundaryConditions.thermal_flux thermal(phi=phi)
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

  model Rectangular1D
    package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1)
      "Refrigerant";

    parameter Integer N=3 "Number of longitudinal sections in which the tube is discretized";
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
    Components.OneDimensional.RectangularChannel1D channel1D(
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
      N=N) annotation (Placement(transformation(extent={{-30,-30},{30,30}})));

    BoundaryConditions.thermal_flux_distributed thermal_distributed(
      N=N,
      phi=phi*ones(N))
      annotation (Placement(transformation(extent={{-40,48},{-4,70}})));
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
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Rectangular1D;

  model Rectangular2D
    package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.5)
      "Refrigerant";

    parameter Integer N_cv=3 "Number of longitudinal sections in which each channel is discretized";
    parameter Integer N_channels=10 "Number of channels";
    parameter Length L "Channel length";
    parameter Length W "Width of the control volume";
    parameter Length H "Height of the control volume";
    parameter Length t_ext "Thickness of external walls";
    parameter Length t_int "Thickness of internal walls";
    parameter MassFlowRate m_flow "Refrigerant mass flow rate";
    parameter Temperature T_in "Refrigerant inlet temperature";

    parameter HeatFlux phi;
    parameter Temperature T_start_solid
      "Temperature of solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Temperature T_start_fluid
      "Temperature of fluid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Pressure P_start=101325
      "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
    parameter MassFlowRate m_flow_start=1
      "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
    parameter MassFraction X_start[Refrigerant.nX]=Refrigerant.reference_X
      "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
    parameter Refrigerant.ThermodynamicState state_start=
      Refrigerant.setState_pTX(P_start, T_start_fluid, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));

    inner Components.Environment environment(
      allowFlowReversal=false, initOpt=DynTherM.Choices.InitOpt.steadyState)
                                                    annotation (Placement(transformation(extent={{60,60},{100,100}})));
    Components.TwoDimensional.RectangularChannels2D channel2D(
      redeclare package Medium = Refrigerant,
      L=L,
      H=H,
      W=W,
      t_int=t_int,
      t_ext=t_ext,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      X_start=X_start,
      state_start=state_start,
      m_flow_start=m_flow_start,
      N_cv=N_cv,
      N_channels=N_channels) annotation (Placement(transformation(extent={{-30,-30},{30,30}})));

    BoundaryConditions.thermal_flux_distributed thermal_distributed(
      N=N_cv,
      phi=phi*ones(N_cv))
      annotation (Placement(transformation(extent={{-40,48},{-4,70}})));
    BoundaryConditions.pressure_sink_distributed pressure_sink_distributed(
        redeclare package Medium = Refrigerant, N=N_channels)
      annotation (Placement(transformation(extent={{66,-14},{94,14}})));
    BoundaryConditions.flow_source_distributed flow_source_distributed(redeclare
        package Medium = Refrigerant, N=N_channels,
      massFlow_di=m_flow/N_channels*ones(N_channels),
      T_di=T_in*ones(N_channels))
      annotation (Placement(transformation(extent={{-94,-14},{-66,14}})));
  equation

    connect(channel2D.solid_surface_north, thermal_distributed.thermal_flux)
      annotation (Line(points={{-22.5,13.8},{-22.5,14},{-22,14},{-22,59}}, color={
            255,127,0}));
    connect(channel2D.outlet, pressure_sink_distributed.inlet) annotation (Line(
          points={{30,0},{48,0},{48,2.66454e-15},{66,2.66454e-15}},    color={0,0,
            0}));
    connect(flow_source_distributed.outlet, channel2D.inlet) annotation (Line(
          points={{-66.42,2.77556e-15},{-48.21,2.77556e-15},{-48.21,-2.22045e-15},
            {-30,-2.22045e-15}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Rectangular2D;

  model Rectangular2D_manual
    package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.5)
      "Refrigerant";

    parameter Integer N=3 "Number of longitudinal sections in which the tube is discretized";
    parameter Modelica.Units.SI.Length L "Channel length";
    parameter Modelica.Units.SI.Length W "Width of the control volume";
    parameter Modelica.Units.SI.Length H "Height of the control volume";
    parameter Modelica.Units.SI.Length t_ext "Thickness of external walls";
    parameter Modelica.Units.SI.Length t_int "Thickness of internal walls";

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

    BoundaryConditions.flow_source west_flow(
      redeclare package Medium = Refrigerant,
      use_in_massFlow=true,
      use_in_T=true) annotation (Placement(transformation(
          extent={{14,14},{-14,-14}},
          rotation=180,
          origin={-80,-60})));
    Modelica.Blocks.Interfaces.RealInput T annotation (Placement(transformation(
            extent={{-130,10},{-90,50}}), iconTransformation(extent={{-106,-30},{-86,
              -10}})));
    Modelica.Blocks.Interfaces.RealInput m annotation (Placement(transformation(
            extent={{-130,-50},{-90,-10}}), iconTransformation(extent={{-106,-70},
              {-86,-50}})));
    BoundaryConditions.pressure_sink pressureSink(redeclare package Medium =
          Refrigerant)
      annotation (Placement(transformation(extent={{68,-12},{92,12}})));
    inner Components.Environment environment(
      allowFlowReversal=false,
      initOpt=DynTherM.Choices.InitOpt.steadyState) annotation (Placement(transformation(extent={{60,60},{100,100}})));
    Components.OneDimensional.RectangularChannel1D west(
      redeclare package Medium = Refrigerant,
      L=L,
      H=H,
      W=W,
      t_north=t_ext,
      t_south=t_ext,
      t_east=t_int,
      t_west=t_int,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      X_start=X_start,
      state_start=state_start,
      m_flow_start=m_flow_start,
      N=N) annotation (Placement(transformation(extent={{-30,-90},{30,-30}})));

    BoundaryConditions.thermal_flux_distributed thermal_distributed(
      N=N,
      phi=phi*ones(N))
      annotation (Placement(transformation(extent={{-78,78},{-42,100}})));
    Components.OneDimensional.RectangularChannel1D internal(
      redeclare package Medium = Refrigerant,
      L=L,
      H=H,
      W=W,
      t_north=t_ext,
      t_south=t_ext,
      t_east=t_int,
      t_west=t_int,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      X_start=X_start,
      state_start=state_start,
      m_flow_start=m_flow_start,
      N=N) annotation (Placement(transformation(extent={{-30,-30},{30,30}})));
    Components.OneDimensional.RectangularChannel1D east(
      redeclare package Medium = Refrigerant,
      L=L,
      H=H,
      W=W,
      t_north=t_ext,
      t_east=t_int,
      t_south=t_ext,
      t_west=t_int,
      T_start_solid=T_start_solid,
      T_start_fluid=T_start_fluid,
      P_start=P_start,
      X_start=X_start,
      state_start=state_start,
      m_flow_start=m_flow_start,
      N=N) annotation (Placement(transformation(extent={{-30,30},{30,90}})));
    BoundaryConditions.flow_source internal_flow(
      redeclare package Medium = Refrigerant,
      use_in_massFlow=true,
      use_in_T=true) annotation (Placement(transformation(
          extent={{14,14},{-14,-14}},
          rotation=180,
          origin={-82,0})));
    BoundaryConditions.flow_source east_flow(
      redeclare package Medium = Refrigerant,
      use_in_massFlow=true,
      use_in_T=true) annotation (Placement(transformation(
          extent={{14,14},{-14,-14}},
          rotation=180,
          origin={-82,60})));
  equation

    connect(internal.outlet, pressureSink.inlet)
      annotation (Line(points={{30,0},{68,0}}, color={0,0,0}));
    connect(west.outlet, pressureSink.inlet)
      annotation (Line(points={{30,-60},{52,-60},{52,0},{68,0}}, color={0,0,0}));
    connect(east.outlet, pressureSink.inlet)
      annotation (Line(points={{30,60},{52,60},{52,0},{68,0}}, color={0,0,0}));
    connect(east.solid_surface_north, thermal_distributed.thermal_flux)
      annotation (Line(points={{-22.5,73.8},{-22.5,74},{-60,74},{-60,89}}, color={
            255,127,0}));
    connect(internal.solid_surface_north, thermal_distributed.thermal_flux)
      annotation (Line(points={{-22.5,13.8},{-22.5,14},{-60,14},{-60,89}}, color={
            255,127,0}));
    connect(west.solid_surface_north, thermal_distributed.thermal_flux)
      annotation (Line(points={{-22.5,-46.2},{-60,-46.2},{-60,89}}, color={255,127,
            0}));
    connect(east.solid_surface_west, internal.solid_surface_east) annotation (
        Line(points={{22.5,73.8},{22.5,30},{-7.5,30},{-7.5,13.8}}, color={255,127,
            0}));
    connect(internal.solid_surface_west, west.solid_surface_east) annotation (
        Line(points={{22.5,13.8},{22.5,-30},{-7.5,-30},{-7.5,-46.2}}, color={255,127,
            0}));
    connect(east_flow.outlet, east.inlet)
      annotation (Line(points={{-68,60},{-30,60}}, color={0,0,0}));
    connect(internal_flow.outlet, internal.inlet) annotation (Line(points={{-68,-3.44169e-15},
            {-49,-3.44169e-15},{-49,0},{-30,0}}, color={0,0,0}));
    connect(west_flow.outlet, west.inlet)
      annotation (Line(points={{-66,-60},{-30,-60}}, color={0,0,0}));
    connect(m, west_flow.in_massFlow) annotation (Line(points={{-110,-30},{-91.2,-30},
            {-91.2,-50.2}}, color={0,0,127}));
    connect(m, internal_flow.in_massFlow) annotation (Line(points={{-110,-30},{-92,
            -30},{-92,9.8},{-93.2,9.8}}, color={0,0,127}));
    connect(m, east_flow.in_massFlow) annotation (Line(points={{-110,-30},{-92,-30},
            {-92,69.8},{-93.2,69.8}}, color={0,0,127}));
    connect(T, internal_flow.in_T) annotation (Line(points={{-110,30},{-84.8,30},{
            -84.8,9.8}}, color={0,0,127}));
    connect(T, east_flow.in_T) annotation (Line(points={{-110,30},{-64,30},{-64,54},
            {-62,54},{-62,74},{-64,74},{-64,76},{-84.8,76},{-84.8,69.8}}, color={0,
            0,127}));
    connect(T, west_flow.in_T) annotation (Line(points={{-110,30},{-62,30},{-62,-42},
            {-82.8,-42},{-82.8,-50.2}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Rectangular2D_manual;
end Distributed;
