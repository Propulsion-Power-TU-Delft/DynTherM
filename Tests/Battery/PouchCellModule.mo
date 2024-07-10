within DynTherM.Tests.Battery;
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
  connect(firewall.inlet, cell_right.Left) annotation (Line(points={{4.8,-50},{
          4.8,-52},{4,-52},{4,-49},{39.6222,-49}},  color={191,0,0}));
  connect(I_charging.y, cell_right.I) annotation (Line(points={{101.2,60},{80,
          60},{80,-20},{110,-20},{110,-40.6667},{99.1556,-40.6667}},
                                            color={0,0,127}));
  connect(cell_left.Right, firewall.outlet) annotation (Line(points={{-46.7778,
          -49},{-46.7778,-50},{-4.8,-50}},          color={191,0,0}));
  connect(I_charging.y, cell_left.I) annotation (Line(points={{101.2,60},{80,60},
          {80,-20},{-20,-20},{-20,-40.6667},{-32.8444,-40.6667}},     color={
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
