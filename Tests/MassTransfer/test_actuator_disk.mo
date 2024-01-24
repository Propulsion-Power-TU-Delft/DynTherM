within DynTherM.Tests.MassTransfer;
model test_actuator_disk "Test of actuator disk model"
  Components.MassTransfer.ActuatorDisk actuatorDisk(
    set_W=true,
    W_fixed(displayUnit="kW") = 1000*(13004/8),
    R_disk=3.72)
    annotation (Placement(transformation(extent={{-40,-40},{40,40}})));
  BoundaryConditions.pressure_sink outlet
    annotation (Placement(transformation(extent={{68,-12},{92,12}})));
  inner Components.Environment environment(phi_amb=0.1)
    annotation (Placement(transformation(extent={{56,56},{98,98}})));
  Modelica.Blocks.Sources.Constant altitude(k=0)
    annotation (Placement(transformation(extent={{20,70},{40,90}})));
  Modelica.Blocks.Sources.Constant V_inf(k=75)
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
equation
  connect(actuatorDisk.outlet, outlet.inlet)
    annotation (Line(points={{40,0},{68,0}}, color={0,0,0}));
  connect(altitude.y, environment.altitude) annotation (Line(points={{41,80},{46,
          80},{46,68.6},{56,68.6}}, color={0,0,127}));
  connect(V_inf.y, environment.V_inf_di) annotation (Line(points={{41,50},{46,
          50},{46,60.2},{56,60.2}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end test_actuator_disk;
