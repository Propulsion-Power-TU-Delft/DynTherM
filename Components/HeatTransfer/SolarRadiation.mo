within DynTherM.Components.HeatTransfer;
model SolarRadiation
  "Model of direct, diffuse and reflected solar irradiance on a planar, arbitrary oriented surface"
  // Reference: ASHRAE Handbook – Fundamentals, chapter 14, 2013.
  // The following models are strictly valid only for clear-sky at ground level
  // Corrections for cloudy-sky and altitude effects should be implemented

  outer DynTherM.Components.Environment environment;
  constant Modelica.Units.SI.Irradiance E_sc=1367 "Solar constant";
  parameter Modelica.Units.SI.Irradiance E_tb_fixed=0.0
    "Fixed value of the beam component of the clear-sky solar irradiance" annotation (Dialog(tab="Coupling with external sw"));
  parameter Modelica.Units.SI.Irradiance E_td_fixed=0.0
    "Fixed value of the diffuse component of the clear-sky solar irradiance" annotation (Dialog(tab="Coupling with external sw"));
  parameter Modelica.Units.SI.Irradiance E_tr_fixed=0.0
    "Fixed value of the ground reflected component of the clear-sky solar irradiance" annotation (Dialog(tab="Coupling with external sw"));
  parameter Modelica.Units.SI.Angle theta_fixed=0.0
    "Fixed value of incidence angle" annotation (Dialog(tab="Coupling with external sw"));
  parameter Real rho_g=0.2 "Ground reflectance";
  parameter Modelica.Units.SI.Angle csi
    "Tilt angle of the surface wrt horizontal";
  parameter Modelica.Units.SI.Angle psi_plus=0
    "Modifier of azimuth angle for this specific section";

  Real ET "Equation of time";
  Real AST "Apparent solar time";
  Modelica.Units.SI.Angle delta "Solar declination";
  Modelica.Units.SI.Angle H "Hour angle";
  Modelica.Units.SI.Angle beta "Solar altitude";
  Modelica.Units.SI.Angle phi "Azimuth angle";
  Modelica.Units.SI.Angle gamma "Surface solar azimuth angle";
  Modelica.Units.SI.Angle theta
    "Incidence angle (difference between sun-earth line and normal to the surface)";
  Real m0 "Relative air mass at ground";
  Real m "Relative air mass corrected for altitude";
  Modelica.Units.SI.Irradiance E0 "Extraterrestrial radiant flux";
  Modelica.Units.SI.Irradiance E_b "Beam normal irradiance";
  Modelica.Units.SI.Irradiance E_d "Diffuse horizontal irradiance";
  Modelica.Units.SI.Irradiance E_tb "Beam component of the clear-sky solar irradiance";
  Modelica.Units.SI.Irradiance E_td "Diffuse component of the clear-sky solar irradiance";
  Modelica.Units.SI.Irradiance E_tr "Ground reflected component of the clear-sky solar irradiance";
  Modelica.Units.SI.Irradiance E "Total clear-sky irradiance";
  CustomInterfaces.IrradiancePort inlet annotation (Placement(transformation(extent={{-10,52},{10,72}})));

protected
  Real Gamma;
  Real cos_phi;
  Real sin_phi;
  Real ab;
  Real ad;
  Real Y;
  Modelica.Units.SI.Angle theta_vertical;

equation
  if environment.use_ext_sw then
    delta = 0.0;
    Gamma = 0.0;
    ET = 0.0;
    AST = 0.0;
    H = 0.0;
    beta = 0.0;
    cos_phi = 0.0;
    sin_phi = 0.0;
    phi = 0.0;
    gamma = 0.0;
    theta_vertical = 0.0;
    E0 = 0.0;
    m0 = 0.0;
    m = 0.0;
    ab = 0.0;
    ad = 0.0;
    E_b = 0.0;
    E_d = 0.0;
    Y = 0.0;
    theta = theta_fixed;
    E_tb = E_tb_fixed;
    E_td = E_td_fixed;
    E_tr = E_tr_fixed;

  else
    delta = 0.40928*Modelica.Math.sin(2*environment.pi*(284 + environment.Day)/365);
    Gamma = 2*environment.pi*(environment.Day - 1)/365;
    ET = 2.2918*(0.0075 + 0.1868*Modelica.Math.cos(Gamma) - 3.2077*
      Modelica.Math.sin(Gamma) - 1.4615*Modelica.Math.cos(2*Gamma) - 4.089*
      Modelica.Math.sin(2*Gamma));
    AST = environment.Hour + (environment.Minute + ET)/60 + 180*(environment.long - environment.long_ref)/(15*environment.pi);
    H = 15*(AST - 12)*environment.pi/180;
    beta = Modelica.Math.asin(Modelica.Math.sin(delta)*Modelica.Math.sin(environment.lat) +
      Modelica.Math.cos(delta)*Modelica.Math.cos(H)*Modelica.Math.cos(environment.lat));
    cos_phi = (Modelica.Math.cos(delta)*Modelica.Math.cos(H)*
      Modelica.Math.sin(environment.lat) - Modelica.Math.sin(delta)*Modelica.Math.cos(environment.lat))/
      Modelica.Math.cos(beta);
    sin_phi = Modelica.Math.cos(delta)*Modelica.Math.sin(H)/
      Modelica.Math.cos(beta);
    phi = Modelica.Math.atan2(sin_phi, cos_phi);
    gamma = phi - (environment.psi + psi_plus);
    theta_vertical = Modelica.Math.acos(Modelica.Math.cos(beta)*
      Modelica.Math.cos(gamma));
    theta = Modelica.Math.acos(Modelica.Math.cos(beta)*Modelica.Math.cos(gamma)*
      Modelica.Math.sin(csi) + Modelica.Math.sin(beta)*Modelica.Math.cos(csi));
    E0 = E_sc*(1 + 0.033*Modelica.Math.cos(2*environment.pi*(environment.Day - 3)/365));
    m0 = 1/(Modelica.Math.sin(beta) + 0.50572*(6.07995 + beta*180/environment.pi)^(-1.6364)); // Eq 16

    // Correction for air mass at low altitude as proportional to pressure ratio G. Aglietti
    m = m0*(((environment.T0 + environment.ISA_plus)-0.0065*environment.Altitude)/
      (environment.T0 + environment.ISA_plus))^5.2561;
    ab = 1.454 - 0.406*environment.tau_b[environment.Month] - 0.268*environment.tau_d[environment.Month] +
      0.021*environment.tau_b[environment.Month]*environment.tau_d[environment.Month];
    ad = 0.507 + 0.205*environment.tau_b[environment.Month] - 0.08*environment.tau_d[environment.Month] -
      0.19*environment.tau_b[environment.Month]*environment.tau_d[environment.Month];
    E_b = E0*Modelica.Math.exp(-environment.tau_b[environment.Month]*m^ab);
    E_d = E0*Modelica.Math.exp(-environment.tau_d[environment.Month]*m^ad);
    Y = max(0.45, 0.55 + 0.437*Modelica.Math.cos(theta_vertical) + 0.313*
      Modelica.Math.cos(theta_vertical)^2);

    if (Modelica.Math.cos(theta) > 0) then
      E_tb = E_b*Modelica.Math.cos(theta);
    else
      E_tb = 0.0;
    end if;

    if (csi*180/environment.pi <= 90) then
      E_td = E_d*(Y*Modelica.Math.sin(csi) + Modelica.Math.cos(csi));
    else
      E_td = E_d*Y*Modelica.Math.sin(csi);
    end if;

    E_tr = (E_b*Modelica.Math.sin(beta) + E_d)*rho_g*(1 - Modelica.Math.cos(csi))/2;

  end if;

  E = E_tb + E_td + E_tr;
  inlet.E_tb = E_tb;
  inlet.E_td = E_td;
  inlet.E_tr = E_tr;
  inlet.theta = theta;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-24,-26},{24,-74}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-2,-2},{2,-22}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-38,-18},{-24,-32},{-20,-28},{-38,-18}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-2,10},{2,-10}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={-38,-50},
          rotation=-90),
        Polygon(
          points={{-9,-7},{5,7},{9,3},{-9,-7}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={-27,-75},
          rotation=360),
        Rectangle(
          extent={{-2,-78},{2,-98}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{9,7},{-5,-7},{-9,-3},{9,7}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={25,-75},
          rotation=-90),
        Rectangle(
          extent={{-2,10},{2,-10}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={38,-50},
          rotation=-90),
        Polygon(
          points={{-9,7},{5,-7},{9,-3},{-9,7}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={27,-25},
          rotation=-90),
        Line(points={{-10,-12},{-10,44}}, color={238,46,47}),
        Line(points={{-10,44},{-16,34}},  color={238,46,47}),
        Line(points={{-10,44},{-4,34}},   color={238,46,47}),
        Line(points={{-30,-12},{-30,44}}, color={238,46,47}),
        Line(points={{-30,44},{-36,34}},  color={238,46,47}),
        Line(points={{-30,44},{-24,34}},  color={238,46,47}),
        Line(points={{10,-12},{10,44}},   color={238,46,47}),
        Line(points={{10,44},{4,34}},     color={238,46,47}),
        Line(points={{10,44},{16,34}},    color={238,46,47}),
        Line(points={{30,-12},{30,44}},   color={238,46,47}),
        Line(points={{30,44},{24,34}},    color={238,46,47}),
        Line(points={{30,44},{36,34}},    color={238,46,47})}),  Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
end SolarRadiation;
