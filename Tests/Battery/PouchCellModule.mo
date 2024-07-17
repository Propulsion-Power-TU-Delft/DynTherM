within DynTherM.Tests.Battery;
model PouchCellModule

  parameter Integer N_cv=10;
  parameter Length W_cell=0.35;
  parameter Length H_cell=0.1;
  parameter Length t_cell=0.01;
  parameter Length t_fw=0.001;
  parameter ElectricCharge C_nom=230400;
  parameter Real SoC_start=0.1;
  parameter Temperature Tstart=298.15;

  Modelica.Blocks.Sources.TimeTable I_charging(table=[0,200; 200,200; 200,200; 500,
        200; 501,120; 700,120; 701,100; 900,100; 901,120; 1000,120; 1001,70; 1200,
        70]) annotation (Placement(transformation(extent={{108,52},{92,68}})));
  inner Components.Environment environment(allowFlowReversal=false, initOpt=
        DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-96,44},{-68,72}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-48,-36},{-32,-20}})));
  Systems.Battery.PouchModuleFirewall module(
    W_cell=W_cell,
    H_cell=H_cell,
    t_cell(displayUnit="mm")=t_cell,
    t_fw(displayUnit="mm")=t_fw,
    t_gap=t_fw,
    C_nom(displayUnit="Ah")=C_nom,
    initOpt=environment.initOpt,
    SoC_start=SoC_start,
    Tstart=Tstart,
    N_cv=N_cv,
    Ns=3,
    Np=4)
    annotation (Placement(transformation(extent={{-28,-26},{40,40}})));
  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={0,40})));
  Modelica.Blocks.Math.Gain gain(k=-1)
    annotation (Placement(transformation(extent={{66,54},{54,66}})));
equation
  connect(module.p, signalCurrent.p) annotation (Line(points={{-14.4,0.4},{
          -14.4,40},{-6,40}}, color={0,0,255}));
  connect(signalCurrent.n, module.n)
    annotation (Line(points={{6,40},{12.8,40},{12.8,0.4}}, color={0,0,255}));
  connect(ground.p, module.p) annotation (Line(points={{-40,-20},{-40,0.4},{
          -14.4,0.4}}, color={0,0,255}));
  connect(gain.u, I_charging.y)
    annotation (Line(points={{67.2,60},{91.2,60}}, color={0,0,127}));
  connect(gain.y, signalCurrent.i)
    annotation (Line(points={{53.4,60},{0,60},{0,47.2}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{120,80}})),                                    Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{120,
            80}})),
    experiment(
      StopTime=1200,
      Interval=1,
      __Dymola_Algorithm="Dassl"));
end PouchCellModule;
