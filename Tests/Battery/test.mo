within DynTherM.Tests.Battery;
model test
  parameter Real SOC=100;
  parameter Modelica.Units.SI.Temperature Tstart=293.15;
  Components.Battery.Cell18650 C11(
    SOC=SOC,
    Tstart=Tstart,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-50,-50},{-30,-30}})));
  Components.Battery.Cell18650 C21(
    SOC=SOC,
    Tstart=Tstart,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));
  Components.Battery.Cell18650 C12(
    SOC=SOC,
    Tstart=Tstart,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  Components.Battery.Cell18650 C22(
    SOC=SOC,
    Tstart=Tstart,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Components.Battery.Cell18650 C13(
    SOC=SOC,
    Tstart=Tstart,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-50,30},{-30,50}})));
  Components.Battery.Cell18650 C23(
    SOC=SOC,
    Tstart=Tstart,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-10,30},{10,50}})));
  Components.Battery.Cell18650 C31(
    SOC=SOC,
    Tstart=Tstart,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{30,-50},{50,-30}})));
  Components.Battery.Cell18650 C32(
    SOC=SOC,
    Tstart=Tstart,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  Components.Battery.Cell18650 C33(
    SOC=SOC,
    Tstart=Tstart,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{30,30},{50,50}})));
  Modelica.Thermal.HeatTransfer.Components.Convection northConvection
    annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={0,74})));
  Modelica.Thermal.HeatTransfer.Components.Convection southConvection
    annotation (Placement(transformation(
        extent={{12,12},{-12,-12}},
        rotation=90,
        origin={0,-74})));
  Modelica.Thermal.HeatTransfer.Components.Convection eastConvection
    annotation (Placement(transformation(
        extent={{12,12},{-12,-12}},
        rotation=180,
        origin={74,0})));
  Modelica.Thermal.HeatTransfer.Components.Convection westConvection
    annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-74,0})));
  Modelica.Blocks.Sources.Constant constW(k=4)
    annotation (Placement(transformation(extent={{-100,-50},{-80,-30}})));
  Modelica.Blocks.Sources.Constant constN(k=4)
    annotation (Placement(transformation(extent={{-50,64},{-30,84}})));
  Modelica.Blocks.Sources.Constant constE(k=4)
    annotation (Placement(transformation(extent={{100,30},{80,50}})));
  Modelica.Blocks.Sources.Constant constS(k=4)
    annotation (Placement(transformation(extent={{50,-84},{30,-64}})));
  BoundaryConditions.thermal N(T=293.15, use_T=true)
    annotation (Placement(transformation(extent={{-12,94},{6,106}})));
  BoundaryConditions.thermal E(T=293.15, use_T=true)
    annotation (Placement(transformation(extent={{88,-6},{106,6}})));
  BoundaryConditions.thermal W(T=293.15, use_T=true)
    annotation (Placement(transformation(extent={{-112,-6},{-94,6}})));
  BoundaryConditions.thermal S(T=293.15, use_T=true)
    annotation (Placement(transformation(extent={{-12,-106},{6,-94}})));
equation
  connect(C13.E, C23.W)
    annotation (Line(points={{-30,40},{-10,40}},color={191,0,0}));
  connect(C12.E, C22.W)
    annotation (Line(points={{-30,0},{-10,0}},color={191,0,0}));
  connect(C11.E, C21.W)
    annotation (Line(points={{-30,-40},{-10,-40}},color={191,0,0}));
  connect(C11.N, C12.S)
    annotation (Line(points={{-40,-30},{-40,-10}}, color={191,0,0}));
  connect(C12.N, C13.S)
    annotation (Line(points={{-40,10},{-40,30}}, color={191,0,0}));
  connect(C22.N, C23.S)
    annotation (Line(points={{0,10},{0,30}},   color={191,0,0}));
  connect(C21.N, C22.S)
    annotation (Line(points={{0,-30},{0,-10}},   color={191,0,0}));
  connect(C31.N, C32.S)
    annotation (Line(points={{40,-30},{40,-10}}, color={191,0,0}));
  connect(C32.N, C33.S)
    annotation (Line(points={{40,10},{40,30}}, color={191,0,0}));
  connect(C23.E, C33.W)
    annotation (Line(points={{10,40},{30,40}}, color={191,0,0}));
  connect(C22.E, C32.W)
    annotation (Line(points={{10,0},{30,0}}, color={191,0,0}));
  connect(C21.E, C31.W)
    annotation (Line(points={{10,-40},{30,-40}}, color={191,0,0}));
  connect(northConvection.solid, C23.N) annotation (Line(points={{-7.77156e-16,62},
          {-7.77156e-16,56},{0,56},{0,50}}, color={191,0,0}));
  connect(southConvection.solid, C21.S) annotation (Line(points={{7.77156e-16,-62},
          {7.77156e-16,-56},{0,-56},{0,-50}}, color={191,0,0}));
  connect(westConvection.solid, C12.W) annotation (Line(points={{-62,-1.44329e-15},
          {-56,-1.44329e-15},{-56,0},{-50,0}}, color={191,0,0}));
  connect(C32.E, eastConvection.solid) annotation (Line(points={{50,0},{56,0},{56,
          1.44329e-15},{62,1.44329e-15}}, color={191,0,0}));
  connect(eastConvection.solid, C31.E) annotation (Line(points={{62,1.44329e-15},
          {56,1.44329e-15},{56,-40},{50,-40}}, color={191,0,0}));
  connect(eastConvection.solid, C33.E) annotation (Line(points={{62,1.44329e-15},
          {56,1.44329e-15},{56,40},{50,40}}, color={191,0,0}));
  connect(C13.N, C23.N) annotation (Line(points={{-40,50},{-40,56},{0,56},{0,50}},
        color={191,0,0}));
  connect(C33.N, C23.N)
    annotation (Line(points={{40,50},{40,56},{0,56},{0,50}}, color={191,0,0}));
  connect(C31.S, C21.S) annotation (Line(points={{40,-50},{40,-56},{0,-56},{0,-50}},
        color={191,0,0}));
  connect(C11.S, C21.S) annotation (Line(points={{-40,-50},{-40,-56},{0,-56},{0,
          -50}}, color={191,0,0}));
  connect(C11.W, C12.W) annotation (Line(points={{-50,-40},{-56,-40},{-56,0},{-50,
          0}}, color={191,0,0}));
  connect(C13.W, C12.W) annotation (Line(points={{-50,40},{-56,40},{-56,0},{-50,
          0}}, color={191,0,0}));
  connect(southConvection.Gc, constS.y)
    annotation (Line(points={{12,-74},{29,-74}}, color={0,0,127}));
  connect(constW.y, westConvection.Gc)
    annotation (Line(points={{-79,-40},{-74,-40},{-74,-12}}, color={0,0,127}));
  connect(constN.y, northConvection.Gc)
    annotation (Line(points={{-29,74},{-12,74}}, color={0,0,127}));
  connect(constE.y, eastConvection.Gc)
    annotation (Line(points={{79,40},{74,40},{74,12}}, color={0,0,127}));
  connect(N.thermal, northConvection.fluid) annotation (Line(points={{0,100},
          {0,93},{7.77156e-16,93},{7.77156e-16,86}}, color={191,0,0}));
  connect(W.thermal, westConvection.fluid) annotation (Line(points={{-100,0},
          {-93,0},{-93,1.44329e-15},{-86,1.44329e-15}}, color={191,0,0}));
  connect(S.thermal, southConvection.fluid) annotation (Line(points={{0,-100},
          {0,-93},{-7.77156e-16,-93},{-7.77156e-16,-86}}, color={191,0,0}));
  connect(eastConvection.fluid, E.thermal) annotation (Line(points={{86,
          -1.44329e-15},{93,-1.44329e-15},{93,0},{100,0}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=1000, __Dymola_Algorithm="Dassl"));
end test;
