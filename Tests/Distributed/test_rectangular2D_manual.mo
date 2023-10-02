within DynTherM.Tests.Distributed;
model test_rectangular2D_manual
  Rectangular2D_manual TwoD(
    L(displayUnit="mm") = 0.4826,
    W(displayUnit="mm") = 0.0559,
    H(displayUnit="mm") = 0.00196,
    t_ext(displayUnit="mm") = 0.001,
    t_int(displayUnit="mm") = 0.0005,
    phi=-13800,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
  Modelica.Blocks.Sources.Constant m(k=0.1)
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  Modelica.Blocks.Sources.Constant T(k=273.15 + 50)
    annotation (Placement(transformation(extent={{-80,-14},{-60,6}})));
equation
  connect(T.y, TwoD.T)
    annotation (Line(points={{-59,-4},{-19.2,-4}}, color={0,0,127}));
  connect(m.y, TwoD.m) annotation (Line(points={{-59,-50},{-40,-50},{-40,-12},
          {-19.2,-12}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end test_rectangular2D_manual;
