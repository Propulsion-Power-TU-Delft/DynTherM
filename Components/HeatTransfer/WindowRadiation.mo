within ThermalManagement.Components.HeatTransfer;
model WindowRadiation
  "0D model of transmitted, absorbed and reflected irradiance through transparent surfaces"
  // Reference: F. Zanghirella, et al. - A Numerical Model to Evaluate the thermal Behaviour of Active Transparent Facades, 2011.
  // Hp: only one internal reflection is considered

  replaceable model Mat=ThermalManagement.Materials.Opticor constrainedby
    ThermalManagement.Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  outer ThermalManagement.Components.Environment environment "Environmental properties";

  parameter Modelica.Units.SI.Area A "Heat transfer surface";
  parameter Modelica.Units.SI.Angle theta_start=0.17 "Refraction angle - starting value" annotation (Dialog(tab="Initialization"));

  Real r_s "Reflection coefficient for s-polarized light";
  Real r_p "Reflection coefficient for p-polarized light";
  Real r_eff "Reflection coefficient for natural light";

  Modelica.Units.SI.Angle theta_r(start=theta_start) "Refraction angle";
  Modelica.Units.SI.Irradiance E_transmitted "Irradiance transmitted through the window";
  Modelica.Units.SI.Irradiance E_reflected "Irradiance reflected by the window";
  Modelica.Units.SI.Irradiance E_absorbed "Irradiance absorbed by the window";

  CustomInterfaces.IrradiancePort outlet
    annotation (Placement(transformation(extent={{-14,-28},{14,0}})));
  CustomInterfaces.IrradiancePort inlet_transmitted annotation (Placement(
        transformation(extent={{-48,20},{-20,48}}), iconTransformation(extent={{
            -48,20},{-20,48}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet_absorbed
    annotation (Placement(transformation(extent={{20,20},{48,48}}),
        iconTransformation(extent={{20,20},{48,48}})));
equation
  // Snell's law
  environment.n_air*Modelica.Math.sin(outlet.theta) = Mat.n*Modelica.Math.sin(theta_r);

  // Fresnel's equations
  r_s = abs(environment.n_air*Modelica.Math.cos(outlet.theta) - Mat.n*Modelica.Math.cos(theta_r))/
        (environment.n_air*Modelica.Math.cos(outlet.theta) + Mat.n*Modelica.Math.cos(theta_r));
  r_p = abs(environment.n_air*Modelica.Math.cos(theta_r) - Mat.n*Modelica.Math.cos(outlet.theta))/
        (environment.n_air*Modelica.Math.cos(theta_r) + Mat.n*Modelica.Math.cos(outlet.theta));
  r_eff = (r_s + r_p)/2;

  if (cos(outlet.theta) > 0) then
    E_reflected = (r_eff + (1 - r_eff)*r_eff*Mat.t^2)*outlet.E_tb;
    E_transmitted = (1 - r_eff)^2*Mat.t*outlet.E_tb;
    E_absorbed = outlet.E_tb - E_reflected - E_transmitted;
  else
    E_reflected = outlet.E_tb;
    E_transmitted = 0;
    E_absorbed = 0;
  end if;

  inlet_transmitted.E_tb = E_transmitted;
  inlet_transmitted.E_td = 0;
  inlet_transmitted.E_tr = 0;
  inlet_transmitted.theta = outlet.theta "due to double refraction";
  inlet_absorbed.Q_flow = -A*E_absorbed;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,20},{100,0}},
          fillColor={85,255,255},
          fillPattern=FillPattern.Backward,
          lineColor={0,0,0}),
        Line(points={{-80,-28},{-80,46}}, color={238,46,47}),
        Line(points={{-60,-28},{-60,46}}, color={238,46,47}),
        Line(points={{-80,46},{-86,36}},  color={238,46,47}),
        Line(points={{-80,46},{-74,36}},  color={238,46,47}),
        Line(points={{-60,46},{-54,36}},  color={238,46,47}),
        Line(points={{-60,46},{-66,36}},  color={238,46,47}),
        Line(points={{60,-28},{60,48}},   color={238,46,47}),
        Line(points={{80,-28},{80,48}},   color={238,46,47}),
        Line(points={{74,-18},{80,-28}}, color={238,46,47}),
        Line(points={{54,-18},{60,-28}}, color={238,46,47}),
        Line(points={{86,-18},{80,-28}}, color={238,46,47}),
        Line(points={{66,-18},{60,-28}}, color={238,46,47})}),   Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
end WindowRadiation;
