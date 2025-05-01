within DynTherM.Tests.Battery;
model PouchCellModuleParallel

  parameter Integer N_cv=10;
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
        70]) annotation (Placement(transformation(extent={{78,62},{62,78}})));
  inner Components.Environment environment(allowFlowReversal=false, initOpt=
        DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{80,20},{114,54}})));
  Components.Electrical.PouchCell cell_right(
    H=H_cell,
    W=W_cell,
    t=t_cell,
    C_nom(displayUnit="Ah") = C_nom,
    initOpt=environment.initOpt,
    SoC_start=SoC_start,
    Tstart=Tstart,
    N=N_cv) annotation (Placement(transformation(extent={{24,-74},{96,-26}})));
  Components.Electrical.PouchCell cell_left(
    H=H_cell,
    W=W_cell,
    t=t_cell,
    C_nom(displayUnit="Ah") = C_nom,
    initOpt=environment.initOpt,
    SoC_start=SoC_start,
    Tstart=Tstart,
    N=N_cv) annotation (Placement(transformation(extent={{-64,-74},{8,-26}})));
  Components.TwoDimensional.WallConductionVertical2D firewall(
    redeclare model MatX = DynTherM.Materials.PolyurethaneFoam,
    redeclare model MatY = DynTherM.Materials.PolyurethaneFoam,
    x=t_fw,
    y=H_cell,
    z=W_cell,
    Tstart=298.15,
    initOpt=environment.initOpt,
    N=N_cv)
    annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={20,-50})));
  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrentScalar
    annotation (Placement(transformation(
        extent={{6,-6},{-6,6}},
        rotation=0,
        origin={20,0})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-88,-36},{-72,-20}})));
  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrentVector
    annotation (Placement(transformation(
        extent={{6,-6},{-6,6}},
        rotation=0,
        origin={-40,54})));
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
    Ns=1,
    Np=2) annotation (Placement(transformation(extent={{-68,-12},{0,54}})));
equation
  connect(cell_left.p, cell_right.p) annotation (Line(points={{0.4,-42.4},{0.4,-20},
          {88.4,-20},{88.4,-42.4}},            color={0,0,255}));
  connect(cell_left.n, cell_right.n) annotation (Line(points={{0.4,-58.4},{0.4,-74},
          {88.4,-74},{88.4,-58.4}},            color={0,0,255}));
  connect(cell_left.p, ground.p) annotation (Line(points={{0.4,-42.4},{0.4,-20},
          {-80,-20}},  color={0,0,255}));
  connect(I_charging.y, signalCurrentVector.i)
    annotation (Line(points={{61.2,70},{-40,70},{-40,61.2}}, color={0,0,127}));
  connect(I_charging.y, signalCurrentScalar.i)
    annotation (Line(points={{61.2,70},{20,70},{20,7.2}}, color={0,0,127}));
  connect(cell_left.p, signalCurrentScalar.n)
    annotation (Line(points={{0.4,-42.4},{0.4,0},{14,0}}, color={0,0,255}));
  connect(signalCurrentScalar.p, cell_right.n) annotation (Line(points={{26,0},{
          100,0},{100,-58.4},{88.4,-58.4}},  color={0,0,255}));
  connect(cell_left.Right, firewall.West)
    annotation (Line(points={{-6,-50},{16.4,-50}}, color={191,0,0}));
  connect(firewall.East, cell_right.Left)
    annotation (Line(points={{23.6,-50},{38.8,-50}}, color={191,0,0}));
  connect(signalCurrentVector.n, module_parallel.p) annotation (Line(points={{
          -46,54},{-54.4,54},{-54.4,14.4}}, color={0,0,255}));
  connect(signalCurrentVector.p, module_parallel.n) annotation (Line(points={{
          -34,54},{-27.2,54},{-27.2,14.4}}, color={0,0,255}));
  connect(module_parallel.p, ground.p) annotation (Line(points={{-54.4,14.4},{
          -80,14.4},{-80,-20}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{120,80}})),                                    Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{120,
            80}})),
    experiment(
      StopTime=1200,
      Interval=1,
      __Dymola_Algorithm="Dassl"));
end PouchCellModuleParallel;
