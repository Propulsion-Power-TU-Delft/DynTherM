within DynTherM.Tests.Distributed;
model WallConduction2D
  Components.TwoDimensional.WallConductionCV2D centralCV(
    x(displayUnit="mm") = 0.03,
    y(displayUnit="mm") = 0.01,
    z(displayUnit="mm") = 1,
    dm=0) annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
  Components.TwoDimensional.WallConductionCV2D southCV(
    x=centralCV.x,
    y=centralCV.y,
    z=centralCV.z,
    dm=0) annotation (Placement(transformation(extent={{-20,-60},{20,-20}})));
  Components.TwoDimensional.WallConductionCV2D northCV(
    x=centralCV.x,
    y=centralCV.y,
    z=centralCV.z,
    dm=0) annotation (Placement(transformation(extent={{-20,20},{20,60}})));
  Components.TwoDimensional.WallConductionCV2D eastCV(
    x=centralCV.x,
    y=centralCV.y,
    z=centralCV.z,
    dm=0) annotation (Placement(transformation(extent={{30,-20},{70,20}})));
  Components.TwoDimensional.WallConductionCV2D westCV(
    x=centralCV.x,
    y=centralCV.y,
    z=centralCV.z,
    dm=0) annotation (Placement(transformation(extent={{-70,-20},{-30,20}})));
  BoundaryConditions.ZeroDimensional.thermal northBC(
    T=323.15,
    use_Q=false,
    use_T=true) annotation (Placement(transformation(extent={{-12,74},{6,86}})));
  BoundaryConditions.ZeroDimensional.thermal southBC(
    T=298.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-12,-86},{6,-74}})));
  BoundaryConditions.ZeroDimensional.thermal westBC(
    T=323.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-102,-6},{-84,6}})));
  BoundaryConditions.ZeroDimensional.thermal eastBC(
    T=298.15,
    use_Q=false,
    use_T=true) annotation (Placement(transformation(extent={{102,-6},{84,6}})));
  Modelica.Blocks.Sources.Constant const(k=0)
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  BoundaryConditions.ZeroDimensional.thermal adiabatic_1(Q=0)
    annotation (Placement(transformation(extent={{-62,34},{-44,46}})));
  BoundaryConditions.ZeroDimensional.thermal adiabatic_4(Q=0)
    annotation (Placement(transformation(extent={{62,34},{44,46}})));
  BoundaryConditions.ZeroDimensional.thermal adiabatic_2(Q=0)
    annotation (Placement(transformation(extent={{-62,-46},{-44,-34}})));
  BoundaryConditions.ZeroDimensional.thermal adiabatic_3(Q=0)
    annotation (Placement(transformation(extent={{62,-46},{44,-34}})));
equation
  connect(westCV.East, centralCV.West)
    annotation (Line(points={{-32,0},{-18,0}}, color={191,0,0}));
  connect(centralCV.East, eastCV.West)
    annotation (Line(points={{18,0},{32,0}}, color={191,0,0}));
  connect(centralCV.North, northCV.South)
    annotation (Line(points={{0,14},{0,26}}, color={191,0,0}));
  connect(southCV.North, centralCV.South)
    annotation (Line(points={{0,-26},{0,-14}}, color={191,0,0}));
  connect(northBC.thermal, northCV.North)
    annotation (Line(points={{0,80},{0,54}}, color={191,0,0}));
  connect(westBC.thermal, westCV.West)
    annotation (Line(points={{-90,0},{-68,0}}, color={191,0,0}));
  connect(eastCV.East, eastBC.thermal)
    annotation (Line(points={{68,0},{90,0}}, color={191,0,0}));
  connect(southBC.thermal, southCV.South)
    annotation (Line(points={{0,-80},{0,-54}}, color={191,0,0}));
  connect(const.y, westCV.Q_int)
    annotation (Line(points={{-79,70},{-58,70},{-58,14}}, color={0,0,127}));
  connect(const.y, northCV.Q_int)
    annotation (Line(points={{-79,70},{-8,70},{-8,54}}, color={0,0,127}));
  connect(const.y, eastCV.Q_int)
    annotation (Line(points={{-79,70},{42,70},{42,14}}, color={0,0,127}));
  connect(const.y, centralCV.Q_int) annotation (Line(points={{-79,70},{-26,70},
          {-26,20},{-8,20},{-8,14}}, color={0,0,127}));
  connect(const.y, southCV.Q_int) annotation (Line(points={{-79,70},{-26,70},{
          -26,-20},{-8,-20},{-8,-26}}, color={0,0,127}));
  connect(adiabatic_2.thermal, westCV.South)
    annotation (Line(points={{-50,-40},{-50,-14}}, color={191,0,0}));
  connect(westCV.North, adiabatic_1.thermal)
    annotation (Line(points={{-50,14},{-50,40}}, color={191,0,0}));
  connect(adiabatic_1.thermal, northCV.West)
    annotation (Line(points={{-50,40},{-18,40}}, color={191,0,0}));
  connect(northCV.East, adiabatic_4.thermal)
    annotation (Line(points={{18,40},{50,40}}, color={191,0,0}));
  connect(adiabatic_4.thermal, eastCV.North)
    annotation (Line(points={{50,40},{50,14}}, color={191,0,0}));
  connect(eastCV.South, adiabatic_3.thermal)
    annotation (Line(points={{50,-14},{50,-40}}, color={191,0,0}));
  connect(adiabatic_3.thermal, southCV.East)
    annotation (Line(points={{50,-40},{18,-40}}, color={191,0,0}));
  connect(southCV.West, adiabatic_2.thermal)
    annotation (Line(points={{-18,-40},{-50,-40}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end WallConduction2D;
