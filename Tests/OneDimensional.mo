within DynTherM.Tests;
package OneDimensional

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
  end test_rectangularCV;

  model test_1D
    OneDimensional OneD(
      L=1,
      R_ext(displayUnit="mm") = 0.05,
      R_int(displayUnit="mm") = 0.045,
      Q_flow(displayUnit="kW") = -1000,
      T_start_solid=323.15,
      T_start_fluid=323.15)
      annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
    Modelica.Blocks.Sources.Constant m(k=5)
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
  end test_1D;

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

    Components.MassTransfer.SourceMassFlow ECSFlow(
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
    Components.MassTransfer.PressureSink pressureSink(
      redeclare package Medium = Refrigerant) annotation (Placement(transformation(extent={{68,-12},{92,12}})));
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

    Components.MassTransfer.SourceMassFlow ECSFlow(
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
    Components.MassTransfer.PressureSink pressureSink(
      redeclare package Medium = Refrigerant) annotation (Placement(transformation(extent={{68,-12},{92,12}})));
    inner Components.Environment environment(
      allowFlowReversal=false,
      initOpt=DynTherM.Choices.InitOpt.steadyState) annotation (Placement(transformation(extent={{60,60},{100,100}})));
    Components.OneDimensional.RectangularCV cv(
      redeclare package Medium = Refrigerant,
      L=L,
      W=W,
      H=H,
      t=t,
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
    connect(thermal.thermal, cv.solid_surface_north) annotation (Line(points={{0,60},
            {0,40},{-18,40},{-18,21}}, color={191,0,0}));
    connect(thermal.thermal, cv.solid_surface_east)
      annotation (Line(points={{0,60},{0,40},{-6,40},{-6,21}}, color={191,0,0}));
    connect(thermal.thermal, cv.solid_surface_south)
      annotation (Line(points={{0,60},{0,40},{6,40},{6,21}}, color={191,0,0}));
    connect(thermal.thermal, cv.solid_surface_west) annotation (Line(points={{0,60},
            {0,40},{17.4,40},{17.4,21}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end RectangularCV;

  model OneDimensional
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

    Components.MassTransfer.SourceMassFlow ECSFlow(
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
    Components.MassTransfer.PressureSink pressureSink(
      redeclare package Medium = Refrigerant) annotation (Placement(transformation(extent={{68,-12},{92,12}})));
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
      pressureDropOpt=DynTherM.Choices.PDropOpt.darcyWeisbach,
      N=N) annotation (Placement(transformation(extent={{-30,-30},{30,30}})));

    BoundaryConditions.thermal_distributed thermal_distributed(N=N, Q=Q_flow*
          ones(N))
      annotation (Placement(transformation(extent={{-18,42},{18,64}})));
  equation
  //   connect(channel1D.solid_surface.ports[1], t1.thermal);
  //   connect(channel1D.solid_surface.ports[2], t2.thermal);
  //   connect(channel1D.solid_surface.ports[3], t3.thermal);

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
  end OneDimensional;
end OneDimensional;
