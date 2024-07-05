within DynTherM.Validation;
package Battery
  model PouchCellPolestar
    package Water = Modelica.Media.Water.ConstantPropertyLiquidWater;
    //package Water = Modelica.Media.Water.StandardWater;
    Components.Battery.PouchCell1D pouchCell1D(
      H=0.1,
      W=0.35,
      t=0.01,
      C_nom(displayUnit="Ah") = 230400,
      initOpt=environment.initOpt,
      SoC_start=0.1,
      Tstart=298.15)
      annotation (Placement(transformation(extent={{-22,26},{54,76}})));
    Modelica.Blocks.Sources.TimeTable I_charging(table=[0,200; 200,200; 200,200; 500,
          200; 501,120; 700,120; 701,100; 900,100; 901,120; 1000,120; 1001,70; 1200,
          70]) annotation (Placement(transformation(extent={{98,52},{82,68}})));
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
      annotation (Placement(transformation(extent={{-6,-66},{46,-14}})));
    BoundaryConditions.flow_source flow_source(
      redeclare package Medium = Water,
      T_nom=298.15,
      massFlow_nom=0.04,
      allowFlowReversal=environment.allowFlowReversal,
      use_in_massFlow=true,
      use_in_T=true)
      annotation (Placement(transformation(extent={{-62,-52},{-38,-28}})));
    BoundaryConditions.pressure_sink pressure_sink(redeclare package Medium =
          Water,                              allowFlowReversal=environment.allowFlowReversal)
      annotation (Placement(transformation(extent={{70,-50},{90,-30}})));
    Modelica.Blocks.Sources.Ramp m_cool(
      height=0.00004,
      duration=12,
      offset=0.04416,
      startTime=20)
      annotation (Placement(transformation(extent={{-88,-28},{-72,-12}})));
    Modelica.Blocks.Sources.Constant T_cool(k=298.15)
      annotation (Placement(transformation(extent={{-88,12},{-72,28}})));
    inner Components.Environment environment(allowFlowReversal=false, initOpt=
          DynTherM.Choices.InitOpt.fixedState)
      annotation (Placement(transformation(extent={{-98,40},{-64,74}})));
  equation
    connect(flow_source.outlet, single_channel.inlet)
      annotation (Line(points={{-38,-40},{-0.8,-40}}, color={0,0,0}));
    connect(single_channel.outlet, pressure_sink.inlet)
      annotation (Line(points={{40.8,-40},{70,-40}}, color={0,0,0}));
    connect(single_channel.solid_surface, pouchCell1D.Bottom)
      annotation (Line(points={{20,-19.2},{20,31},{20.2222,31}},
                                                      color={191,0,0}));
    connect(m_cool.y, flow_source.in_massFlow) annotation (Line(points={{-71.2,-20},
            {-59.6,-20},{-59.6,-31.6}}, color={0,0,127}));
    connect(T_cool.y, flow_source.in_T) annotation (Line(points={{-71.2,20},{-52.4,
            20},{-52.4,-31.6}}, color={0,0,127}));
    connect(I_charging.y, pouchCell1D.I) annotation (Line(points={{81.2,60},{54,
            60},{54,59.3333},{53.1556,59.3333}},
                                             color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},
              {100,80}})),                                         Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{100,80}})),
      experiment(
        StopTime=1200,
        Interval=1,
        __Dymola_Algorithm="Dassl"));
  end PouchCellPolestar;

  model PouchCellModule
    package Water = Modelica.Media.Water.StandardWater;

    parameter Integer N_cv=10;
    parameter Length W_cell=0.35;
    parameter Length H_cell=0.1;
    parameter Length t_cell=0.01;
    parameter Length t_fw=0.001;

    Modelica.Blocks.Sources.TimeTable I_charging(table=[0,200; 200,200; 200,200; 500,
          200; 501,120; 700,120; 701,100; 900,100; 901,120; 1000,120; 1001,70; 1200,
          70]) annotation (Placement(transformation(extent={{118,52},{102,68}})));
    inner Components.Environment environment(allowFlowReversal=false, initOpt=
          DynTherM.Choices.InitOpt.fixedState)
      annotation (Placement(transformation(extent={{-86,34},{-42,78}})));
    Components.Battery.PouchCell1D cell_right(
      H=H_cell,
      W=W_cell,
      t=t_cell,
      C_nom(displayUnit="Ah") = 230400,
      initOpt=environment.initOpt,
      SoC_start=0.1,
      Tstart=298.15,
      N=N_cv)
      annotation (Placement(transformation(extent={{24,-74},{100,-24}})));
    Components.Battery.PouchCell1D cell_left(
      H=H_cell,
      W=W_cell,
      t=t_cell,
      C_nom(displayUnit="Ah") = 230400,
      initOpt=environment.initOpt,
      SoC_start=0.1,
      Tstart=298.15,
      N=N_cv)
      annotation (Placement(transformation(extent={{-108,-74},{-32,-24}})));
    Components.OneDimensional.WallConduction1D firewall(
      redeclare model Mat = DynTherM.Materials.PolyurethaneFoam,
      t=t_fw,
      A=W_cell*H_cell,
      Tstart=298.15,
      initOpt=environment.initOpt,
      N=N_cv)                                           annotation (Placement(
          transformation(
          extent={{-20,-16},{20,16}},
          rotation=-90,
          origin={1.77636e-15,-50})));
    Systems.Battery.PouchModuleFirewall battery_module(
      W_cell=W_cell,
      H_cell=H_cell,
      t_cell(displayUnit="mm") = t_cell,
      t_fw(displayUnit="mm") = t_fw,
      C_nom(displayUnit="Ah") = 230400,
      initOpt=environment.initOpt,
      SoC_start=0.1,
      Tstart=298.15,
      N_cv=N_cv,
      N_parallel=1,
      N_series=2)
      annotation (Placement(transformation(extent={{-32,-14},{48,66}})));
  equation
    connect(firewall.inlet, cell_right.Left) annotation (Line(points={{4.8,-50},
            {4.8,-52},{4,-52},{4,-49},{39.6222,-49}}, color={191,0,0}));
    connect(I_charging.y, cell_right.I) annotation (Line(points={{101.2,60},{80,
            60},{80,-20},{110,-20},{110,-40.6667},{99.1556,-40.6667}},
                                              color={0,0,127}));
    connect(cell_left.Right, firewall.outlet) annotation (Line(points={{
            -46.7778,-49},{-46.7778,-50},{-4.8,-50}}, color={191,0,0}));
    connect(I_charging.y, cell_left.I) annotation (Line(points={{101.2,60},{80,
            60},{80,-20},{-20,-20},{-20,-40.6667},{-32.8444,-40.6667}}, color={
            0,0,127}));
    connect(I_charging.y, battery_module.I)
      annotation (Line(points={{101.2,60},{4.8,60},{4.8,45.2}},
                                                          color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -80},{120,80}})),                                    Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{120,
              80}})),
      experiment(
        StopTime=1200,
        Interval=1,
        __Dymola_Algorithm="Dassl"));
  end PouchCellModule;

  model PouchCellModuleCooling
    package Water = Modelica.Media.Water.ConstantPropertyLiquidWater;

    parameter Integer N_cv=10;
    parameter Length W_cell=0.35;
    parameter Length H_cell=0.1;
    parameter Length t_cell=0.01;
    parameter Length t_fw=0.001;

    Modelica.Blocks.Sources.TimeTable I_charging(table=[0,200; 200,200; 200,200; 500,
          200; 501,120; 700,120; 701,100; 900,100; 901,120; 1000,120; 1001,70; 1200,
          70]) annotation (Placement(transformation(extent={{118,52},{102,68}})));
    inner Components.Environment environment(allowFlowReversal=true,  initOpt=
          DynTherM.Choices.InitOpt.fixedState)
      annotation (Placement(transformation(extent={{-86,34},{-42,78}})));
    Systems.Battery.PouchModuleFirewall battery_module(
      W_cell=W_cell,
      H_cell=H_cell,
      t_cell(displayUnit="mm") = t_cell,
      t_fw(displayUnit="mm") = t_fw,
      C_nom(displayUnit="Ah") = 230400,
      initOpt=environment.initOpt,
      SoC_start=0.1,
      Tstart=298.15,
      N_cv=N_cv,
      N_parallel=1,
      N_series=10)
      annotation (Placement(transformation(extent={{-32,-14},{48,66}})));
    Components.Battery.ParallelRectangularColdPlate parallelRectangularColdPlate(
      redeclare package Medium = Water,
      L=0.109,
      W_plate=W_cell,
      V_inertia=1e-6,
        allowFlowReversal=environment.allowFlowReversal,
      initOpt=environment.initOpt,
      T_start_plate=298.15,
      T_start_fluid=298.15,
      P_start=400000,
      m_flow_start=1,
      u_start=20,
      rho_start(displayUnit="kg/m3") = 1,                N_cv=battery_module.N_cells)
      annotation (Placement(transformation(extent={{-24,-54},{24,-6}})));
    BoundaryConditions.flow_source flow_source(
      redeclare package Medium = Water,
      P_nom=400000,
      T_nom=298.15,
      massFlow_nom=1,
      allowFlowReversal=environment.allowFlowReversal,
      use_in_massFlow=false,
      use_in_T=false)
      annotation (Placement(transformation(extent={{-84,-42},{-60,-18}})));
    BoundaryConditions.pressure_sink pressure_sink(
      redeclare package Medium = Water,
      allowFlowReversal=environment.allowFlowReversal,
      use_ambient=false,
      P_di=400000,
      T_di=298.15)
      annotation (Placement(transformation(extent={{70,-40},{90,-20}})));
  equation
    connect(I_charging.y, battery_module.I)
      annotation (Line(points={{101.2,60},{4.8,60},{4.8,45.2}},
                                                          color={0,0,127}));
    connect(parallelRectangularColdPlate.upper_surface, battery_module.Bottom)
      annotation (Line(points={{0,-13.68},{0,-0.4}}, color={191,0,0}));
    connect(flow_source.outlet, parallelRectangularColdPlate.inlet)
      annotation (Line(points={{-60,-30},{-24,-30}}, color={0,0,0}));
    connect(parallelRectangularColdPlate.outlet, pressure_sink.inlet)
      annotation (Line(points={{24,-30},{70,-30}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -80},{120,80}})),                                    Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{120,
              80}})),
      experiment(
        StopTime=1200,
        Interval=1,
        __Dymola_Algorithm="Dassl"));
  end PouchCellModuleCooling;
end Battery;
