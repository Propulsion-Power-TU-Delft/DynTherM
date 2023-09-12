within ThermalManagement.BoundaryConditions;
model thermal "Model to impose heat flow rate and temperature"
  extends ThermalManagement.Base.tin1;
  parameter Modelica.Units.SI.Temperature T=273.15 "Temperature"
    annotation (Dialog(tab="Boundary conditions", enable=use_T));
  parameter Modelica.Units.SI.HeatFlowRate Q=1e3 "Heat flow rate"
    annotation (Dialog(tab="Boundary conditions", enable=use_Q));
  parameter Boolean use_Q = false "True if heat flow rate is given"  annotation (Dialog(tab="Boundary conditions"));
  parameter Boolean use_T = false "True if temperature is given" annotation (Dialog(tab="Boundary conditions"));
  parameter Boolean use_in_T = false "Use connector input for the temperature" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  parameter Boolean use_in_Q = false "Use connector input for the heat flow rate" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  Modelica.Blocks.Interfaces.RealInput in_T if use_in_T annotation (Placement(
        transformation(
        origin={-60,64},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={54,20})));
  Modelica.Blocks.Interfaces.RealInput in_Q if use_in_Q annotation (Placement(
        transformation(
        origin={0,90},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={54,-20})));
protected
  Modelica.Blocks.Interfaces.RealInput in_T0 annotation (Placement(
        transformation(
        origin={-60,64},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,60})));
  Modelica.Blocks.Interfaces.RealInput in_Q0 annotation (Placement(
        transformation(
        origin={0,90},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
equation
  //Boundary equations
  if use_T then
    thermal.T = T;
  end if;
  if use_Q then
    thermal.Q_flow = Q;
  end if;

  in_T0 = thermal.T;
  in_Q0 = thermal.Q_flow;

  // Connect protected connectors to public conditional connectors
  connect(in_T, in_T0);
  connect(in_Q, in_Q0);
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
            {120,20}})),                  Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
          extent={{66,8},{82,-8}},
          lineColor={0,0,0},
          textString="AT")}));
end thermal;
