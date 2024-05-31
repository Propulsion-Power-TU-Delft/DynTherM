within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model HelicopterFlyingNASA
  "Convection during low forward speed (helicopter) for rotor-induced surfaces using wake-based induced velocity"

  extends BaseClassExternal;

  parameter Length R_rotor "Main rotor radius";
  parameter Velocity V_wind
    "Wind speed (positive as headwind)";
  parameter Force T "Engine Thrust";
  parameter Length L_nose
    "Distance from nose of helicopter";
  parameter Real R=287.058 "Specific gas constant";
  //parameter Real Cp=0 "Pressure coefficient: approximately zero for passenger section around the fuselage";
  parameter Length R_rel=R_rel
    "Radial distance of panel with respect to main shaft";
  parameter Angle psi=psi
    "Rotor azimuth angle of panel";
  parameter AngularVelocity Omega_rotor=30
    "Main rotor angular speed";
  parameter Angle alpha=0 "Angle of attack";

  CoefficientOfHeatTransfer ht_alt
    "Heat transfer coefficient based on ASHRAE HVAC";
  CoefficientOfHeatTransfer ht_turb
    "Heat transfer coefficient based on ASHRAE turbulent flat plate theory";
  Velocity V_inf "Helicopter velocity";
  Velocity V "Surface airflow velocity";
  Velocity V_ind(start=5) "Induced rotor velocity";
  Velocity V_h(start=10)
    "Induced rotor velocity at hover";
  Pressure P_star "Pressure surrounding the fuselage";
  Temperature T_aw "Recovery temperature";
  Real r "Recovery factor for turbulent boundary layer";
  ReynoldsNumber Re_star(start=1e8)
    "Reynolds number used in the heat transfer correlation";
  PrandtlNumber Pr_star
    "Prandtl number used in the heat transfer correlation";
  Real Cp;
  Real u(start=5);
  Real u_1;
  Real r_n "Normalised rotor dimension";
  Angle xsi(start=pi/6) "Wake skew angle";

protected
  Medium.ThermodynamicState state_amb;
  Medium.ThermodynamicState state_aw;
  Medium.ThermodynamicState state_star;
  Real gamma_amb;
  Real nu_factor;
  PrandtlNumber Pr_aw(start=0.75);
  Temperature T_star;

equation
  state_amb = Medium.setState_pTX(environment.P_amb, environment.T_amb, environment.X_amb); // thermodynamic ambient state
  state_aw = Medium.setState_pTX(P_star, T_aw, environment.X_amb); // thermodynamics state at wall
  gamma_amb = Medium.isentropicExponent(state_amb);
  V_inf = environment.Mach_inf*sqrt(gamma_amb*R*environment.T_amb);
  V_h = sqrt(T/(pi*R_rotor^2*2*Medium.density(state_aw))); //estimated basic induced velocity during hover
  V = sqrt((V_inf+V_wind)^2 + V_ind^2); // vectored velocity over the plate

  r_n = R_rel/R_rotor;
  V_h^4 = u^4 + 2*u^3*V_inf*sin(alpha)+u^2*V_inf^2;
  V_inf/u = 2*tan(xsi/2)/cos(xsi+alpha);
  u_1 = u*tan(xsi/2);
  V_ind = u + u_1*r_n*cos(psi);

  P_star = environment.P_amb + Cp*Medium.density(state_star)*V_inf^2/2;
  Pr_aw = Medium.specificHeatCapacityCp(state_aw)*Medium.dynamicViscosity(state_aw)/
    Medium.thermalConductivity(state_aw);
  r = Pr_aw^(1/3);
  T_aw = environment.T_amb*(1 + r*(gamma_amb - 1)/2*environment.Mach_inf^2);
  T_star = (T_aw + environment.T_amb)/2 + 0.22*(T_aw - environment.T_amb); // use an intermediate temperature to evaluate the heat transfer coefficient
  state_star = Medium.setState_pTX(P_star, T_star, environment.X_amb);
  Re_star = Medium.density(state_star)*V*L_nose/Medium.dynamicViscosity(state_star);
  Pr_star = Medium.specificHeatCapacityCp(state_star)*Medium.dynamicViscosity(state_star)/
    Medium.thermalConductivity(state_star);
  Cp = 1 - (V/V_inf)^2;

  if Re_star >= 5e5 then
    nu_factor = 0.0296;
  else
    nu_factor = 0.037;
  end if;

  if Re_star >= 10e7 then
    ht = ht_alt;
  else
    ht = ht_turb;
  end if;

  ht_turb = nu_factor*Re_star^(4/5)*Pr_star^(1/3);
  ht_alt = Medium.density(state_star)*Medium.specificHeatCapacityCp(state_star)*
    V*0.185*((Modelica.Math.log10(Re_star))^(-2.584))*(Pr_star^(-2/3));

  T_out = T_aw;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Legacy code: to be double-checked before usage</p>
<p><b>Reference</b>:</p>
<p>[1] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;HVAC&nbsp;Applications,&nbsp;chapter&nbsp;13</p>
<p>[2] NACA&nbsp;-&nbsp;Evaluation&nbsp;of&nbsp;Induced&nbsp;Field</p>
</html>"));
end HelicopterFlyingNASA;
