within DynTherM.BoundaryConditions;
model mechanical "Model to impose torque and rotational speed"
  extends DynTherM.Base.min1;
  parameter Modelica.Units.SI.Torque M=0 "Torque"
    annotation (Dialog(tab="Boundary conditions", enable=use_M));
  parameter Modelica.Units.SI.AngularVelocity omega=3000 "Rotational speed"
    annotation (Dialog(tab="Boundary conditions", enable=use_omega));
  parameter Modelica.Units.SI.Power W=1e3 "Power"
    annotation (Dialog(tab="Boundary conditions", enable=use_W));
  parameter Boolean use_M = false "True if torque is given"  annotation (Dialog(tab="Boundary conditions"));
  parameter Boolean use_omega = false "True if rotational speed is given"  annotation (Dialog(tab="Boundary conditions"));
  parameter Boolean use_W = false "True if mechanical power is given"  annotation (Dialog(tab="Boundary conditions"));
  parameter Boolean use_in_M = false "Use connector input for the torque" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  parameter Boolean use_in_omega = false "Use connector input for the rotational speed" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  parameter Boolean use_in_W = false "Use connector input for the mechanical power" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  Modelica.Blocks.Interfaces.RealInput in_M if use_in_M annotation (Placement(
        transformation(
        origin={-60,64},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={54,20})));
  Modelica.Blocks.Interfaces.RealInput in_omega if use_in_omega annotation (Placement(
        transformation(
        origin={0,90},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={54,-20})));
  Modelica.Blocks.Interfaces.RealInput in_W if use_in_W annotation (Placement(
        transformation(
        origin={0,90},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={54,-20})));
protected
  Modelica.Blocks.Interfaces.RealInput in_M0 annotation (Placement(
        transformation(
        origin={-60,64},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,60})));
  Modelica.Blocks.Interfaces.RealInput in_omega0 annotation (Placement(
        transformation(
        origin={0,90},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
  Modelica.Blocks.Interfaces.RealInput in_W0 annotation (Placement(
        transformation(
        origin={0,90},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));

equation
  //Boundary equations
  if use_M then
    mechanical.M = M;
  end if;
  if use_omega then
    mechanical.omega = omega;
  end if;
  if use_W then
    mechanical.M*mechanical.omega =  W;
  end if;

  in_M0 = mechanical.M;
  in_omega0 = mechanical.omega;
  in_W0 = mechanical.M*mechanical.omega;

  // Connect protected connectors to public conditional connectors
  connect(in_M, in_M0);
  connect(in_omega, in_omega0);
  connect(in_W, in_W0);
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
            {120,20}}),        graphics), Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
          extent={{66,8},{82,-8}},
          lineColor={0,0,0},
          textString="AM")}));
end mechanical;
