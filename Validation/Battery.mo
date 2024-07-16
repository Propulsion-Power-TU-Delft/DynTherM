within DynTherM.Validation;
package Battery
  model PouchCellPolestar
    package Water = Modelica.Media.Water.StandardWater;
    Components.Battery.PouchCell1D pouchCell1D(
      H=0.1,
      W=0.35,
      t=0.01,
      C_nom(displayUnit="Ah") = 230400,
      initOpt=environment.initOpt,
      SoC_start=0.1,
      Tstart=298.15)
      annotation (Placement(transformation(extent={{-42,26},{34,76}})));
    Modelica.Blocks.Sources.TimeTable I_charging(table=[0,200; 200,200; 200,200; 500,
          200; 501,120; 700,120; 701,100; 900,100; 901,120; 1000,120; 1001,70; 1200,
          70]) annotation (Placement(transformation(extent={{78,42},{62,58}})));
    Components.OneDimensional.CircularCV single_channel(
      redeclare model Mat = DynTherM.Materials.Aluminium,
      redeclare package Medium = Water,
      allowFlowReversal=environment.allowFlowReversal,
      initOpt=environment.initOpt,
      L=0.3,
      R_ext(displayUnit="mm") = 0.0051,
      R_int(displayUnit="mm") = 0.005,
      T_start_solid=298.15,
      T_start_fluid=298.15,
      m_flow_start=0.044,
      u_start=1)
      annotation (Placement(transformation(extent={{-24,-64},{24,-16}})));
    BoundaryConditions.flow_source flow_source(
      redeclare package Medium = Water,
      T_nom=298.15,
      massFlow_nom=0.04,
      allowFlowReversal=environment.allowFlowReversal,
      use_in_massFlow=true,
      use_in_T=true)
      annotation (Placement(transformation(extent={{-70,-52},{-46,-28}})));
    BoundaryConditions.pressure_sink pressure_sink(redeclare package Medium =
          Water,                              allowFlowReversal=environment.allowFlowReversal)
      annotation (Placement(transformation(extent={{50,-50},{70,-30}})));
    Modelica.Blocks.Sources.Ramp m_cool(
      height=0.00004,
      duration=12,
      offset=0.04416,
      startTime=20)
      annotation (Placement(transformation(extent={{-98,-28},{-82,-12}})));
    Modelica.Blocks.Sources.Constant T_cool(k=298.15)
      annotation (Placement(transformation(extent={{-98,12},{-82,28}})));
    inner Components.Environment environment(allowFlowReversal=false, initOpt=
          DynTherM.Choices.InitOpt.fixedState)
      annotation (Placement(transformation(extent={{-98,40},{-64,74}})));
    Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
        Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=-90,
          origin={40,50})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{80,36},{100,56}})));
  equation
    connect(flow_source.outlet, single_channel.inlet)
      annotation (Line(points={{-46,-40},{-19.2,-40}},color={0,0,0}));
    connect(single_channel.outlet, pressure_sink.inlet)
      annotation (Line(points={{19.2,-40},{50,-40}}, color={0,0,0}));
    connect(single_channel.solid_surface, pouchCell1D.Bottom)
      annotation (Line(points={{0,-20.8},{0,31},{0.22222,31}},
                                                      color={191,0,0}));
    connect(m_cool.y, flow_source.in_massFlow) annotation (Line(points={{-81.2,
            -20},{-68,-20},{-68,-31.6},{-67.6,-31.6}},
                                        color={0,0,127}));
    connect(T_cool.y, flow_source.in_T) annotation (Line(points={{-81.2,20},{
            -60.4,20},{-60.4,-31.6}},
                                color={0,0,127}));
    connect(pouchCell1D.p, signalCurrent.p) annotation (Line(points={{25.9778,
            58.9167},{25.9778,58},{40,58}}, color={0,0,255}));
    connect(pouchCell1D.n, signalCurrent.n) annotation (Line(points={{25.9778,
            42.25},{25.9778,42},{40,42}}, color={0,0,255}));
    connect(signalCurrent.i, I_charging.y)
      annotation (Line(points={{49.6,50},{61.2,50}}, color={0,0,127}));
    connect(pouchCell1D.p, ground.p) annotation (Line(points={{25.9778,58.9167},
            {25.9778,72},{90,72},{90,56}}, color={0,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},
              {100,80}})),                                         Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{100,80}})),
      experiment(
        StopTime=1200,
        Interval=1,
        __Dymola_Algorithm="Dassl"));
  end PouchCellPolestar;

end Battery;
