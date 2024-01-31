within DynTherM.Tests.MassTransfer;
model test_actuator_disk "Test of actuator disk model"
  Components.MassTransfer.ActuatorDisk actuatorDisk(
    set_W=true,
    W_fixed(displayUnit="kW") = 1000*(13004/8),
    R_disk=3.72)
    annotation (Placement(transformation(extent={{-40,-40},{40,40}})));
  BoundaryConditions.pressure_sink outlet
    annotation (Placement(transformation(extent={{68,-12},{92,12}})));
  inner Components.Environment environment(phi_amb=0.1,
    V_inf_di=75,
    use_di_Mach_inf=false,
    use_di_V_inf=true)
    annotation (Placement(transformation(extent={{56,56},{98,98}})));
equation
  connect(actuatorDisk.outlet, outlet.inlet)
    annotation (Line(points={{40,0},{68,0}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end test_actuator_disk;
