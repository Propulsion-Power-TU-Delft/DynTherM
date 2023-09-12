within ThermalManagement.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model HelicopterFlyingComplex
  "FlyingComplex: Convection during low forward speed (helicopter) for rotor-induced surfaces using rotor based pressure distribution"
  // Reference: ASHRAE Handbook - HVAC Applications, chapter 13, A. Kusyomov - Simulation Flow around Helicopter Fuselage
  extends BaseClassExternal;
  parameter Modelica.Units.SI.Length R_rotor "Main rotor radius";
  parameter Modelica.Units.SI.Velocity V_wind
    "Wind speed (positive as headwind)";
  parameter Modelica.Units.SI.Force T "Engine Thrust";
  parameter Modelica.Units.SI.Length L_nose
    "Distance from nose of helicopter";
  parameter Real R=287.058 "Specific gas constant";
  //parameter Real Cp=0 "Pressure coefficient: approximately zero for passenger section around the fuselage";
  parameter Modelica.Units.SI.Length R_rel=R_rel
    "Radial distance of panel with respect to main shaft";
  parameter Modelica.Units.SI.Angle psi=psi
    "Rotor azimuth angle of panel";
  parameter Modelica.Units.SI.AngularVelocity Omega_rotor=30
    "Main rotor angular speed";
  parameter Modelica.Units.SI.Angle alpha=0 "Angle of attack";
  Modelica.Units.SI.CoefficientOfHeatTransfer ht_alt
    "Heat transfer coefficient based on ASHRAE HVAC";
  Modelica.Units.SI.CoefficientOfHeatTransfer ht_turb
    "Heat transfer coefficient based on ASHRAE turbulent flat plate theory";
  Modelica.Units.SI.Velocity V_inf "Helicopter velocity";
  Modelica.Units.SI.Velocity V "Surface airflow velocity";
  Modelica.Units.SI.Velocity V_h "Induced rotor velocity at hover";
  Modelica.Units.SI.Pressure P_star "Pressure surrounding the fuselage";
  Modelica.Units.SI.Temperature T_aw "Recovery temperature";
  Real r "Recovery factor for turbulent boundary layer";
  Modelica.Units.SI.ReynoldsNumber Re_star(start=1e8)
    "Reynolds number used in the heat transfer correlation";
  Modelica.Units.SI.PrandtlNumber Pr_star
    "Prandtl number used in the heat transfer correlation";
  Real Cp;
  Real mu;
  Real C_t;
  Real r_n;
  Real f_r;
  Real C_r;
  Real y_r;
  Real f_s;
  Real v_ya;
  Real v_xa;
  Real mu_v;
  Real k_d=tan(pi/4);
  Real B;
  Real y_s;
  Real sigma=0.1 "Solidity";
  Modelica.Units.SI.Pressure dP "Change in pressure around fuselage";
  constant Real pi=Modelica.Constants.pi;
protected
  Medium.ThermodynamicState state_amb;
  Medium.ThermodynamicState state_aw;
  Medium.ThermodynamicState state_star;
  Real gamma_amb;
  Real nu_factor;
  Modelica.Units.SI.PrandtlNumber Pr_aw(start=0.75);
  Modelica.Units.SI.Temperature T_star;
equation
  state_amb = Medium.setState_pTX(environment.P_amb, environment.T_amb, environment.X_amb); // thermodynamic ambient state
  state_aw = Medium.setState_pTX(P_star, T_aw, environment.X_amb); // thermodynamics state at wall
  gamma_amb = Medium.isentropicExponent(state_amb);
  V_inf = environment.Mach_inf*sqrt(gamma_amb*R*environment.T_amb);
  V_h = sqrt(T/(pi*R_rotor^2*2*Medium.density(state_aw))); //estimated basic induced velocity during hover
  V = sqrt((V_inf+V_wind)^2 + V_h^2); // vectored velocity over the plate

  mu = V_inf/(R_rotor*Omega_rotor);
  C_t = T/(pi*Medium.density(state_amb)*R_rotor^4*Omega_rotor^2);
  r_n = R_rel/R_rotor;
  f_r = r_n^2*(2-r_n^2-r_n^4);
  C_r = 1.989*V_inf*(-cos(alpha)+sqrt(cos(alpha)^2+1.27*C_t/mu^2));
  y_r = C_r*f_r;

  f_s = f_r*(r_n^(-1)-25/13*r_n);
  v_ya = 0.5*V_inf*(-cos(alpha)+sqrt(cos(alpha)^2+C_t/mu^2));
  v_xa = 0.5*v_ya;
  mu_v = mu + v_xa/(Omega_rotor*R_rotor);
  B = (8*mu_v*(1+k_d^2) + 2*pi*sigma*k_d)/((1+k_d^2)*(4*mu_v+2*pi*sigma*k_d));
  y_s = C_r*B*mu_v*f_s;

  dP = Medium.density(state_star)*(y_r^2/2+y_s^2/4+y_r*V_inf*cos(alpha));
  P_star = environment.P_amb - dP;
  Pr_aw = Medium.specificHeatCapacityCp(state_aw)*Medium.dynamicViscosity(state_aw)/
    Medium.thermalConductivity(state_aw);
  r = Pr_aw^(1/3);
  T_aw = environment.T_amb*(1 + r*(gamma_amb - 1)/2*environment.Mach_inf^2);
  T_star = (T_aw + environment.T_amb)/2 + 0.22*(T_aw - environment.T_amb); // use an intermediate temperature to evaluate the heat transfer coefficient
  state_star = Medium.setState_pTX(P_star, T_star, environment.X_amb);
  Re_star = Medium.density(state_star)*V*L_nose/Medium.dynamicViscosity(state_star);
  Pr_star = Medium.specificHeatCapacityCp(state_star)*Medium.dynamicViscosity(state_star)/
    Medium.thermalConductivity(state_star);
  Cp = 2*(P_star-environment.P_amb)/(Medium.density(state_star)*V_inf^2);
  if Re_star >= 5e5 then
    nu_factor = 0.0296;
  else
    nu_factor = 0.037;
  end if;
  ht_turb = nu_factor*Re_star^(4/5)*Pr_star^(1/3);
  ht_alt = Medium.density(state_star)*Medium.specificHeatCapacityCp(state_star)*
      V*0.185*((Modelica.Math.log10(Re_star))^(-2.584))*(Pr_star^(-2/3));
  if Re_star >= 10e7 then
    ht = ht_alt;
  else
    ht = ht_turb;
  end if;
  T_out = T_aw;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HelicopterFlyingComplex;
