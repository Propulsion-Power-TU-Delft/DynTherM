within DynTherM.Tests.MassTransfer;
model CircularPipeBend "Simple test of circular pipe bend"
  package Refrigerant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.1)
    "Refrigerant";

  inner Components.Environment environment(
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.fixedState)
    annotation (Placement(transformation(extent={{60,60},{100,100}})));
  BoundaryConditions.ZeroDimensional.flow_source source_Rr_1(
    redeclare package Medium = Refrigerant,
    P_nom=200000,
    T_nom=298.15,
    massFlow_nom=2,
    use_in_massFlow=false,
    use_in_T=false,
    P_start=200000) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,-40})));
  BoundaryConditions.ZeroDimensional.pressure_sink sink_Rr1(redeclare package
      Medium = Refrigerant)
    annotation (Placement(transformation(extent={{68,-52},{92,-28}})));
  Components.MassTransfer.CircularPipeBend bend_Rr1(
    redeclare package Medium = Refrigerant,
    allowFlowReversal=false,
    u_start=2,
    rho_start(displayUnit="kg/m3") = 1000,
    L(displayUnit="m") = 0.5,
    D(displayUnit="mm") = 0.0254,
    R_bend(displayUnit="mm") = 0.01651,
    Roughness(displayUnit="m") = 0.2e-06)
    annotation (Placement(transformation(extent={{-8,-64},{40,-16}})));

  BoundaryConditions.ZeroDimensional.flow_source source_Rr5(
    redeclare package Medium = Refrigerant,
    P_nom=200000,
    T_nom=298.15,
    massFlow_nom=2,
    use_in_massFlow=false,
    use_in_T=false,
    P_start=200000) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,0})));
  BoundaryConditions.ZeroDimensional.pressure_sink sink_Rr5(redeclare package
      Medium = Refrigerant)
    annotation (Placement(transformation(extent={{68,-12},{92,12}})));
  Components.MassTransfer.CircularPipeBend bend_Rr5(
    redeclare package Medium = Refrigerant,
    allowFlowReversal=false,
    u_start=2,
    rho_start(displayUnit="kg/m3") = 1000,
    L(displayUnit="m") = 0.5,
    D(displayUnit="mm") = 0.0254,
    R_bend(displayUnit="mm") = 0.0635,
    Roughness(displayUnit="m") = 0.2e-06)
    annotation (Placement(transformation(extent={{-8,-24},{40,24}})));
  BoundaryConditions.ZeroDimensional.flow_source source_Rr20(
    redeclare package Medium = Refrigerant,
    P_nom=200000,
    T_nom=298.15,
    massFlow_nom=2,
    use_in_massFlow=false,
    use_in_T=false,
    P_start=200000) annotation (Placement(transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,40})));
  BoundaryConditions.ZeroDimensional.pressure_sink sink_Rr20(redeclare package
      Medium = Refrigerant)
    annotation (Placement(transformation(extent={{68,28},{92,52}})));
  Components.MassTransfer.CircularPipeBend bend_Rr20(
    redeclare package Medium = Refrigerant,
    allowFlowReversal=false,
    u_start=2,
    rho_start(displayUnit="kg/dm3") = 1000000,
    L(displayUnit="m") = 0.5,
    D(displayUnit="mm") = 0.0254,
    R_bend(displayUnit="mm") = 0.254,
    Roughness(displayUnit="m") = 0.2e-06)
    annotation (Placement(transformation(extent={{-8,16},{40,64}})));
equation
  connect(source_Rr_1.outlet, bend_Rr1.inlet)
    annotation (Line(points={{-46,-40},{-8,-40}}, color={0,0,0}));
  connect(bend_Rr1.outlet, sink_Rr1.inlet)
    annotation (Line(points={{40,-40},{68,-40}}, color={0,0,0}));
  connect(source_Rr5.outlet, bend_Rr5.inlet)
    annotation (Line(points={{-46,0},{-8,0}}, color={0,0,0}));
  connect(bend_Rr5.outlet, sink_Rr5.inlet)
    annotation (Line(points={{40,0},{68,0}}, color={0,0,0}));
  connect(source_Rr20.outlet, bend_Rr20.inlet)
    annotation (Line(points={{-46,40},{-8,40}}, color={0,0,0}));
  connect(bend_Rr20.outlet, sink_Rr20.inlet)
    annotation (Line(points={{40,40},{68,40}}, color={0,0,0}));
  annotation (Documentation(info="<html>

</html>"), experiment(StopTime=5000, Interval=0.1));
end CircularPipeBend;
