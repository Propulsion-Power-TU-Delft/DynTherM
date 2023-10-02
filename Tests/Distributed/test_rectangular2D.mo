within DynTherM.Tests.Distributed;
model test_rectangular2D
  Rectangular2D TwoD(
    N_cv=3,
    N_channels=3,
    L(displayUnit="m") = 5,
    W(displayUnit="mm") = 0.01,
    H(displayUnit="mm") = 0.002,
    t_ext(displayUnit="mm") = 0.001,
    t_int(displayUnit="mm") = 0.0004,
    m_flow=10/242,
    T_in=323.15,
    phi=-13800,
    T_start_solid=323.15,
    T_start_fluid=323.15)
    annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end test_rectangular2D;
