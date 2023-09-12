within ThermalManagement.Tests.Battery;
model CylindricalCell
  BoundaryConditions.thermal N(T=273.15, use_T=true)
    annotation (Placement(transformation(extent={{-12,94},{6,106}})));
  Modelica.Thermal.HeatTransfer.Components.Convection northConvection
    annotation (Placement(transformation(
        extent={{-16,-15.5},{16,15.5}},
        rotation=90,
        origin={0.5,56})));
  Modelica.Blocks.Sources.Constant constN(k=1)
    annotation (Placement(transformation(extent={{-50,46},{-30,66}})));
  Components.Battery.Cell18650 cell18650(
    SOC=100,
    Tstart=293.15,
    initOpt=ThermalManagement.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{-26,-26},{26,26}})));
  Modelica.Thermal.HeatTransfer.Components.Convection southConvection
    annotation (Placement(transformation(
        extent={{16,-15.5},{-16,15.5}},
        rotation=90,
        origin={0.5,-56})));
  Modelica.Thermal.HeatTransfer.Components.Convection eastConvection
    annotation (Placement(transformation(
        extent={{16,-15.5},{-16,15.5}},
        rotation=180,
        origin={56.5,0})));
  Modelica.Thermal.HeatTransfer.Components.Convection westConvection
    annotation (Placement(transformation(
        extent={{-16,-15.5},{16,15.5}},
        rotation=180,
        origin={-55.5,0})));
  BoundaryConditions.thermal W(T=273.15, use_T=true)
    annotation (Placement(transformation(extent={{-112,-6},{-94,6}})));
  BoundaryConditions.thermal S(T=273.15, use_T=true)
    annotation (Placement(transformation(extent={{-12,-106},{6,-94}})));
  BoundaryConditions.thermal E(T=273.15, use_T=true)
    annotation (Placement(transformation(extent={{88,-6},{106,6}})));
  Modelica.Blocks.Sources.Constant constW(k=4)
    annotation (Placement(transformation(extent={{-90,-50},{-70,-30}})));
  Modelica.Blocks.Sources.Constant constE(k=2)
    annotation (Placement(transformation(extent={{90,-50},{70,-30}})));
  Modelica.Blocks.Sources.Constant constS(k=3)
    annotation (Placement(transformation(extent={{-50,-66},{-30,-46}})));
equation
  connect(northConvection.fluid, N.thermal) annotation (Line(points={{0.5,72},
          {0.5,99},{0,99},{0,100}}, color={191,0,0}));
  connect(constN.y, northConvection.Gc)
    annotation (Line(points={{-29,56},{-15,56}}, color={0,0,127}));
  connect(cell18650.N, northConvection.solid)
    annotation (Line(points={{0,26},{0.5,26},{0.5,40}}, color={191,0,0}));
  connect(southConvection.solid, cell18650.S) annotation (Line(points={{0.5,
          -40},{0.5,-27},{0,-27},{0,-26}}, color={191,0,0}));
  connect(westConvection.solid, cell18650.W) annotation (Line(points={{-39.5,
          -1.9984e-15},{-32.75,-1.9984e-15},{-32.75,0},{-26,0}}, color={191,0,
          0}));
  connect(cell18650.E, eastConvection.solid) annotation (Line(points={{26,0},
          {33.25,0},{33.25,1.9984e-15},{40.5,1.9984e-15}}, color={191,0,0}));
  connect(eastConvection.fluid, E.thermal) annotation (Line(points={{72.5,
          -1.9984e-15},{84,-1.9984e-15},{84,0},{100,0}}, color={191,0,0}));
  connect(W.thermal, westConvection.fluid) annotation (Line(points={{-100,0},
          {-85.75,0},{-85.75,1.9984e-15},{-71.5,1.9984e-15}}, color={191,0,0}));
  connect(S.thermal, southConvection.fluid) annotation (Line(points={{0,-100},
          {0.5,-100},{0.5,-72}}, color={191,0,0}));
  connect(constW.y, westConvection.Gc) annotation (Line(points={{-69,-40},{
          -55.5,-40},{-55.5,-15.5}}, color={0,0,127}));
  connect(constS.y, southConvection.Gc)
    annotation (Line(points={{-29,-56},{-15,-56}}, color={0,0,127}));
  connect(constE.y, eastConvection.Gc) annotation (Line(points={{69,-40},{
          56.5,-40},{56.5,-15.5}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=1000, __Dymola_Algorithm="Dassl"));
end CylindricalCell;
