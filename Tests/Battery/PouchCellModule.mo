within DynTherM.Tests.Battery;
model PouchCellModule

  parameter Integer N_cv=3;
  parameter Length W_cell=0.35;
  parameter Length H_cell=0.1;
  parameter Length t_cell=0.01;
  parameter Length t_fw=0.001;
  parameter Length t_resin=0.0005;
  parameter Length t_frame=0.005;
  parameter ElectricCharge C_nom=230400;
  parameter Real SoC_start=0.1;
  parameter Temperature Tstart=298.15;

  Modelica.Blocks.Sources.TimeTable I_charging(table=[0,200; 200,200; 200,200; 500,
        200; 501,120; 700,120; 701,100; 900,100; 901,120; 1000,120; 1001,70; 1200,
        70]) annotation (Placement(transformation(extent={{68,52},{52,68}})));
  inner Components.Environment environment(allowFlowReversal=false, initOpt=
        DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-76,44},{-48,72}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-48,-36},{-32,-20}})));
  Systems.Battery.PouchModuleParallel module_parallel(
    W_cell=W_cell,
    H_cell=H_cell,
    t_cell(displayUnit="mm") = t_cell,
    t_fw(displayUnit="mm") = t_fw,
    t_resin=t_resin,
    t_frame=t_frame,
    C_nom(displayUnit="Ah") = C_nom,
    initOpt=environment.initOpt,
    SoC_start=SoC_start,
    Tstart=Tstart,
    N_cv=N_cv,
    Ns=3,
    Np=4) annotation (Placement(transformation(extent={{-28,-26},{40,40}})));
  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent1
    annotation (Placement(transformation(
        extent={{6,-6},{-6,6}},
        rotation=0,
        origin={0,40})));
equation
  connect(signalCurrent1.n, module_parallel.p)
    annotation (Line(points={{-6,40},{-14.4,40},{-14.4,0.4}},
                                                            color={0,0,255}));
  connect(signalCurrent1.p, module_parallel.n)
    annotation (Line(points={{6,40},{12.8,40},{12.8,0.4}},  color={0,0,255}));
  connect(module_parallel.p, ground.p)
    annotation (Line(points={{-14.4,0.4},{-40,0.4},{-40,-20}},
                                                             color={0,0,255}));
  connect(I_charging.y, signalCurrent1.i)
    annotation (Line(points={{51.2,60},{0,60},{0,47.2}},   color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-80},
            {80,80}})),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-80},{80,80}})),
    experiment(
      StopTime=1200,
      Interval=1,
      __Dymola_Algorithm="Dassl"));
end PouchCellModule;
