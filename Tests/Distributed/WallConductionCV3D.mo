within DynTherM.Tests.Distributed;
model WallConductionCV3D
  Components.ThreeDimensional.WallConductionCV3D centralCV(
    x(displayUnit="mm") = 0.03,
    y(displayUnit="mm") = 0.01,
    z(displayUnit="mm") = 1,
    dm=0) annotation (Placement(transformation(extent={{-20,-18},{20,22}})));
  Components.ThreeDimensional.WallConductionCV3D southCV(
    x=centralCV.x,
    y=centralCV.y,
    z=centralCV.z,
    dm=0) annotation (Placement(transformation(extent={{-20,-58},{20,-18}})));
  Components.ThreeDimensional.WallConductionCV3D northCV(
    x=centralCV.x,
    y=centralCV.y,
    z=centralCV.z,
    dm=0) annotation (Placement(transformation(extent={{-20,22},{20,62}})));
  Components.ThreeDimensional.WallConductionCV3D eastCV(
    x=centralCV.x,
    y=centralCV.y,
    z=centralCV.z,
    dm=0) annotation (Placement(transformation(extent={{30,-18},{70,22}})));
  Components.ThreeDimensional.WallConductionCV3D westCV(
    x=centralCV.x,
    y=centralCV.y,
    z=centralCV.z,
    dm=0) annotation (Placement(transformation(extent={{-70,-18},{-30,22}})));
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
  BoundaryConditions.ZeroDimensional.thermal adiabatic_1(Q=0)
    annotation (Placement(transformation(extent={{-62,34},{-44,46}})));
  BoundaryConditions.ZeroDimensional.thermal adiabatic_2(Q=0)
    annotation (Placement(transformation(extent={{-62,-46},{-44,-34}})));
  BoundaryConditions.ZeroDimensional.thermal adiabatic_3(Q=0)
    annotation (Placement(transformation(extent={{62,-46},{44,-34}})));
  BoundaryConditions.ZeroDimensional.thermal adiabatic_4(Q=0)
    annotation (Placement(transformation(extent={{62,34},{44,46}})));
  Modelica.Blocks.Sources.Constant const(k=0)
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  BoundaryConditions.ZeroDimensional.thermal frontBC(
    T=323.15,
    use_Q=false,
    use_T=true)
    annotation (Placement(transformation(extent={{-42,-36},{-24,-24}})));
  BoundaryConditions.ZeroDimensional.thermal rearBC(
    T=298.15,
    use_Q=false,
    use_T=true) annotation (Placement(transformation(extent={{18,24},{36,36}})));
equation
  connect(centralCV.North, northCV.South)
    annotation (Line(points={{0,6},{0,34}},  color={191,0,0}));
  connect(southCV.North, centralCV.South)
    annotation (Line(points={{0,-34},{0,-6}},  color={191,0,0}));
  connect(northBC.thermal, northCV.North)
    annotation (Line(points={{0,80},{0,46}}, color={191,0,0}));
  connect(westBC.thermal, westCV.West)
    annotation (Line(points={{-90,0},{-62,0}}, color={191,0,0}));
  connect(eastCV.East, eastBC.thermal)
    annotation (Line(points={{62,0},{90,0}}, color={191,0,0}));
  connect(southBC.thermal, southCV.South)
    annotation (Line(points={{0,-80},{0,-46}}, color={191,0,0}));
  connect(westCV.North, adiabatic_1.thermal)
    annotation (Line(points={{-50,6},{-50,40}},  color={191,0,0}));
  connect(adiabatic_2.thermal, westCV.South)
    annotation (Line(points={{-50,-40},{-50,-6}},  color={191,0,0}));
  connect(adiabatic_3.thermal, eastCV.South)
    annotation (Line(points={{50,-40},{50,-6}},  color={191,0,0}));
  connect(adiabatic_4.thermal, eastCV.North)
    annotation (Line(points={{50,40},{50,6}},  color={191,0,0}));
  connect(const.y, westCV.Q_int)
    annotation (Line(points={{-79,70},{-42,70},{-42,14}}, color={0,0,127}));
  connect(const.y, northCV.Q_int)
    annotation (Line(points={{-79,70},{8,70},{8,54}},   color={0,0,127}));
  connect(const.y, eastCV.Q_int)
    annotation (Line(points={{-79,70},{58,70},{58,14}}, color={0,0,127}));
  connect(const.y, centralCV.Q_int) annotation (Line(points={{-79,70},{-26,70},
          {-26,20},{8,20},{8,14}},  color={0,0,127}));
  connect(const.y, southCV.Q_int) annotation (Line(points={{-79,70},{-26,70},{
          -26,-20},{8,-20},{8,-26}},
                                   color={0,0,127}));
  connect(adiabatic_1.thermal, northCV.West)
    annotation (Line(points={{-50,40},{-12,40}}, color={191,0,0}));
  connect(northCV.East, adiabatic_4.thermal)
    annotation (Line(points={{12,40},{50,40}}, color={191,0,0}));
  connect(southCV.East, adiabatic_3.thermal)
    annotation (Line(points={{12,-40},{50,-40}}, color={191,0,0}));
  connect(adiabatic_2.thermal, southCV.West)
    annotation (Line(points={{-50,-40},{-12,-40}}, color={191,0,0}));
  connect(centralCV.East, eastCV.West)
    annotation (Line(points={{12,0},{38,0}}, color={191,0,0}));
  connect(westCV.East, centralCV.West)
    annotation (Line(points={{-38,0},{-12,0}}, color={191,0,0}));
  connect(frontBC.thermal, centralCV.Front)
    annotation (Line(points={{-30,-30},{-30,-8},{-8,-8}}, color={191,0,0}));
  connect(centralCV.Rear, rearBC.thermal)
    annotation (Line(points={{8,8},{30,8},{30,30}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end WallConductionCV3D;
