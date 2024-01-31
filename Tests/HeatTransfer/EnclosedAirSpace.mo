within DynTherM.Tests.HeatTransfer;
model EnclosedAirSpace
  Components.HeatTransfer.EnclosedAirSpace enclosedAirSpace(
    w(displayUnit="mm") = 0.007,
    h=0.33,
    l=0.23,
    delta=1.553343034275)
    annotation (Placement(transformation(extent={{-36,-36},{36,36}})));
  BoundaryConditions.thermal int(
    T=253.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-20,50},{10,70}})));
  BoundaryConditions.thermal int1(
    T=293.15,
    use_Q=false,
    use_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-20,-70},{10,-50}})));
  inner Components.Environment environment(
    altitude_di(displayUnit="km") = 11000,
    V_inf_di=0.78,
    ISA_plus=0,
    phi_amb=0.2)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
equation
  enclosedAirSpace.P = 85000;
  enclosedAirSpace.X = environment.X_amb;
  connect(enclosedAirSpace.inlet, int.thermal) annotation (Line(points={{
          4.44089e-16,12.24},{0,12.24},{0,60}},
                                 color={191,0,0}));
  connect(int1.thermal, enclosedAirSpace.outlet) annotation (Line(points={{0,-60},
          {4.44089e-16,-60},{4.44089e-16,-12.24}},
                                               color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end EnclosedAirSpace;
