within DynTherM.Tests;
package TwoDimensional
  model TubeSection "Simple test of component pipe"
    Modelica.Blocks.Sources.Constant m_fromMix(k=1)
      annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
    Modelica.Blocks.Sources.Constant T_fromMix(k=273.15 + 50)
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    Modelica.Blocks.Sources.Constant Xw_fromMix(k=0)
      annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
    TwoDimensional.TubeSection_sys tmp(
      L=1,
      R_ext=0.1,
      R_int=0.09,
      Q_flow(displayUnit="kW") = 10000,
      T_start_solid=323.15,
      T_start_fluid=323.15)
      annotation (Placement(transformation(extent={{-20,-16},{20,24}})));
  equation
    connect(T_fromMix.y, tmp.T_fromMix)
      annotation (Line(points={{-59,0},{-19.2,0}}, color={0,0,127}));
    connect(Xw_fromMix.y, tmp.Xw_fromMix) annotation (Line(points={{-59,40},{
            -40,40},{-40,8},{-19.2,8}}, color={0,0,127}));
    connect(m_fromMix.y, tmp.m_fromMix) annotation (Line(points={{-59,-40},{-40,
            -40},{-40,-8},{-19.2,-8}}, color={0,0,127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false)),
      Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=10,
        __Dymola_NumberOfIntervals=100000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"));
  end TubeSection;

  model TubeSection_sys "Simple test of component pipe"
    parameter Modelica.Units.SI.Length L "Tube length";
    parameter Modelica.Units.SI.Length R_ext "Tube external radius";
    parameter Modelica.Units.SI.Length R_int "Tube internal radius";
    parameter Modelica.Units.SI.HeatFlowRate Q_flow;
    parameter Modelica.Units.SI.Temperature T_start_solid
      "Temperature of solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Temperature T_start_fluid
      "Temperature of fluid part - start value" annotation (Dialog(tab="Initialization"));

    Modelica.Blocks.Interfaces.RealInput T_fromMix annotation (Placement(
          transformation(extent={{-130,0},{-90,40}}),  iconTransformation(
            extent={{-106,-30},{-86,-10}})));
    inner Components.Environment environment(
      phi_amb=0.01,
      phi_amb_ground=0.01,
      allowFlowReversal=false,
      initOpt=DynTherM.Choices.InitOpt.fixedState)
      annotation (Placement(transformation(extent={{60,60},{100,100}})));
    Components.MassTransfer.SourceMassFlow ECSFlow(
      use_in_massFlow=true,
      use_in_T=true,
      use_in_Xw=true)
                     annotation (Placement(transformation(
          extent={{14,14},{-14,-14}},
          rotation=180,
          origin={-80,-30})));
    Modelica.Blocks.Interfaces.RealInput Xw_fromMix annotation (Placement(
          transformation(extent={{-130,30},{-90,70}}),  iconTransformation(
            extent={{-106,10},{-86,30}})));
    Modelica.Blocks.Interfaces.RealInput m_fromMix annotation (Placement(
          transformation(extent={{-130,-30},{-90,10}}),iconTransformation(
            extent={{-106,-70},{-86,-50}})));
    Components.MassTransfer.PressureSink pressureSink(use_ambient=false)
      annotation (Placement(transformation(extent={{76,-42},{100,-18}})));
    Components.MassTransfer.Pipe pipe(
      option=DynTherM.Choices.PDropOpt.darcyWeisbach,
      T_start=T_start_fluid,
      L=L,
      D=R_int/2)
             annotation (Placement(transformation(extent={{-24,-54},{24,-6}})));
    Components.TwoDimensional.TubeSection tubeSection(
      L=L,
      R_ext=R_ext,
      R_int=R_int,
      T_start=T_start_solid,
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{-20,20},{20,60}})));
    BoundaryConditions.thermal N(
      T=273.15,
      Q=Q_flow,
      use_Q=true,
      use_T=false)
      annotation (Placement(transformation(extent={{-12,74},{6,86}})));
    BoundaryConditions.thermal E(
      T=273.15,
      Q=Q_flow,
      use_Q=true,
      use_T=false)
      annotation (Placement(transformation(extent={{28,34},{46,46}})));
    BoundaryConditions.thermal S(
      T=273.15,
      Q=Q_flow,
      use_Q=true,
      use_T=false)
      annotation (Placement(transformation(extent={{-12,-6},{6,6}})));
    BoundaryConditions.thermal W(
      T=273.15,
      Q=Q_flow,
      use_Q=true,
      use_T=false)
      annotation (Placement(transformation(extent={{-52,34},{-34,46}})));
  equation
    connect(m_fromMix, ECSFlow.in_massFlow)
      annotation (Line(points={{-110,-10},{-91.2,-10},{-91.2,-20.2}},
                                                             color={0,0,127}));
    connect(T_fromMix, ECSFlow.in_T)
      annotation (Line(points={{-110,20},{-82.8,20},{-82.8,-20.2}},
                                                             color={0,0,127}));
    connect(ECSFlow.outlet, pipe.inlet)
      annotation (Line(points={{-66,-30},{-24,-30}}, color={0,0,0}));
    connect(pipe.outlet, pressureSink.inlet)
      annotation (Line(points={{24,-30},{76,-30}}, color={0,0,0}));
    connect(Xw_fromMix, ECSFlow.in_Xw) annotation (Line(points={{-110,50},{-74,50},
            {-74,-20.2},{-74.4,-20.2}},     color={0,0,127}));
    connect(tubeSection.int, pipe.thermalPort) annotation (Line(points={{0,40},{-8,
            40},{-8,-20.88},{0,-20.88}}, color={191,0,0}));
    connect(N.thermal, tubeSection.N)
      annotation (Line(points={{0,80},{0,60}}, color={191,0,0}));
    connect(W.thermal, tubeSection.W)
      annotation (Line(points={{-40,40},{-20,40}}, color={191,0,0}));
    connect(tubeSection.E,E. thermal)
      annotation (Line(points={{20,40},{40,40}}, color={191,0,0}));
    connect(S.thermal, tubeSection.S)
      annotation (Line(points={{0,0},{0,20}}, color={191,0,0}));
    annotation (Documentation(info="<html>

</html>"),   experiment(StopTime=5000, Interval=0.1));
  end TubeSection_sys;

  model TubeSection_sys2 "Simple test of component pipe"
    package Refrigerant = Media.IncompressibleTableBased.MEG(X=0.1) "Refrigerant";

    parameter Modelica.Units.SI.Length L=1 "Tube length";
    parameter Modelica.Units.SI.Length R_ext=0.1 "Tube external radius";
    parameter Modelica.Units.SI.Length R_int=0.09 "Tube internal radius";
    parameter Modelica.Units.SI.HeatFlowRate Q_flow=10e3;
    parameter Modelica.Units.SI.Temperature T_start_solid=273.15+50
      "Temperature of solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Temperature T_start_fluid=273.15+50
      "Temperature of fluid part - start value" annotation (Dialog(tab="Initialization"));

    inner Components.Environment environment(
      phi_amb=0.01,
      phi_amb_ground=0.01,
      allowFlowReversal=false,
      initOpt=DynTherM.Choices.InitOpt.steadyState)
      annotation (Placement(transformation(extent={{60,60},{100,100}})));
    Components.MassTransfer.Pipe pipe(
      redeclare package Medium = Refrigerant,
      option=DynTherM.Choices.PDropOpt.darcyWeisbach,
      T_start=T_start_fluid,
      L=L,
      D=R_int/2)
             annotation (Placement(transformation(extent={{-24,-54},{24,-6}})));
    Components.TwoDimensional.TubeSection tubeSection(
      L=L,
      R_ext=R_ext,
      R_int=R_int,
      T_start=T_start_solid,
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{-20,20},{20,60}})));
    BoundaryConditions.thermal N(
      T=273.15,
      Q=Q_flow,
      use_Q=true,
      use_T=false)
      annotation (Placement(transformation(extent={{-12,74},{6,86}})));
    BoundaryConditions.thermal E(
      T=273.15,
      Q=Q_flow,
      use_Q=true,
      use_T=false)
      annotation (Placement(transformation(extent={{28,34},{46,46}})));
    BoundaryConditions.thermal S(
      T=273.15,
      Q=Q_flow,
      use_Q=true,
      use_T=false)
      annotation (Placement(transformation(extent={{-12,-6},{6,6}})));
    BoundaryConditions.thermal W(
      T=273.15,
      Q=Q_flow,
      use_Q=true,
      use_T=false)
      annotation (Placement(transformation(extent={{-52,34},{-34,46}})));
  equation
    connect(tubeSection.int, pipe.thermalPort) annotation (Line(points={{0,40},{-8,
            40},{-8,-20.88},{0,-20.88}}, color={191,0,0}));
    connect(N.thermal, tubeSection.N)
      annotation (Line(points={{0,80},{0,60}}, color={191,0,0}));
    connect(W.thermal, tubeSection.W)
      annotation (Line(points={{-40,40},{-20,40}}, color={191,0,0}));
    connect(tubeSection.E,E. thermal)
      annotation (Line(points={{20,40},{40,40}}, color={191,0,0}));
    connect(S.thermal, tubeSection.S)
      annotation (Line(points={{0,0},{0,20}}, color={191,0,0}));
    connect(inlet.node, pipe.inlet)
      annotation (Line(points={{-60,-30},{-24,-30}}, color={0,0,0}));
    connect(pipe.outlet, outlet.node)
      annotation (Line(points={{24,-30},{60,-30}}, color={0,0,0}));
    annotation (Documentation(info="<html>

</html>"),   experiment(StopTime=5000, Interval=0.1));
  end TubeSection_sys2;

  model TubeSection_sys3 "Simple test of component pipe"
    package Refrigerant = Media.IncompressibleTableBased.MEG(X=0.1) "Refrigerant";

    parameter Modelica.Units.SI.Length L=1 "Tube length";
    parameter Modelica.Units.SI.Length R_ext=0.1 "Tube external radius";
    parameter Modelica.Units.SI.Length R_int=0.09 "Tube internal radius";
    parameter Modelica.Units.SI.HeatFlowRate Q_flow=10e3;
    parameter Modelica.Units.SI.Temperature T_start_solid=273.15+50
      "Temperature of solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Temperature T_start_fluid=273.15+50
      "Temperature of fluid part - start value" annotation (Dialog(tab="Initialization"));

    inner Components.Environment environment(
      phi_amb=0.01,
      phi_amb_ground=0.01,
      allowFlowReversal=false,
      initOpt=DynTherM.Choices.InitOpt.steadyState)
      annotation (Placement(transformation(extent={{60,60},{100,100}})));
    annotation (Documentation(info="<html>

</html>"),   experiment(StopTime=5000, Interval=0.1));
  end TubeSection_sys3;
end TwoDimensional;
