within DynTherM.BoundaryConditions;
model radiation "Model to impose irradiance"
  extends DynTherM.Base.rin1;
  input Modelica.Units.SI.Irradiance E "Incident irradiance" annotation (Dialog(tab="Boundary conditions", enable=use_E));
  input Modelica.Units.SI.Angle theta "Incidence angle" annotation (Dialog(tab="Boundary conditions", enable=use_theta));
  parameter Boolean use_E = false "True if irradiance is given" annotation (Dialog(tab="Boundary conditions"));
  parameter Boolean use_in_E = false "Use connector input for the irradiance" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  parameter Boolean use_theta = false "True if theta is given" annotation (Dialog(tab="Boundary conditions"));
  parameter Boolean use_in_theta = false "Use connector input for the theta" annotation (Dialog(group="External inputs"), choices(checkBox=true));

  Modelica.Blocks.Interfaces.RealInput in_E if use_in_E annotation (Placement(
        transformation(
        origin={60,48},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={54,20})));
  Modelica.Blocks.Interfaces.RealInput in_theta if use_in_theta annotation (Placement(
        transformation(
        origin={80,48},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={54,-20})));
protected
  Modelica.Blocks.Interfaces.RealInput in_E0 annotation (Placement(
        transformation(
        origin={60,32},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,60})));
  Modelica.Blocks.Interfaces.RealInput in_theta0 annotation (Placement(
        transformation(
        origin={80,32},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,60})));

equation
  radiative.E_td = 0;
  radiative.E_tr = 0;

  //Boundary equations
  // Need to update if-equations to allow for different combo of inputs/parameter (input E, parameter theta or viceversa)
  if use_E then
    radiative.E_tb = E;
  elseif (use_E and use_theta and (Modelica.Math.cos(theta) > 0)) then
    radiative.theta = theta;
    radiative.E_tb = E*Modelica.Math.cos(theta);
  end if;

  if use_theta then
    radiative.theta = theta;
  end if;

  in_E0 = radiative.E_tb;
  in_theta0 = radiative.theta;

  // Connect protected connectors to public conditional connectors
  connect(in_E, in_E0);
  connect(in_theta, in_theta0);
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
            {120,20}})),                  Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
          extent={{64,8},{80,-8}},
          lineColor={0,0,0},
          textString="AT"), Rectangle(
          extent={{88,12},{114,-12}},
          lineColor={244,125,35},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid)}),
    Documentation(info="<html>
<p>Irradiance&nbsp;is&nbsp;imposed&nbsp;as&nbsp;beam&nbsp;component, whereas&nbsp;the&nbsp;diffused&nbsp;and&nbsp;ground-reflected&nbsp;components&nbsp;are&nbsp;set&nbsp;to&nbsp;zero&nbsp;by&nbsp;default</p>
</html>"));
end radiation;
