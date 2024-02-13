within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model AircraftFlying
  "Convection along the fuselage during flight (aircraft)"
  extends BaseClassExternal;

  parameter Real Cp=0 "Pressure coefficient: approximately zero for passenger section around the fuselage";
  parameter Real coeff=1 "Fraction of cylinder with active heat transfer";
  parameter Modelica.Units.SI.Length L "Fuselage element length";
  parameter Modelica.Units.SI.Length R_ext "Fuselage radius";
  parameter Modelica.Units.SI.Length L_nose
    "Average distance among current fuselage section and fuselage nose";

  Modelica.Units.SI.Velocity V_inf "Aircraft velocity";
  Modelica.Units.SI.Pressure P_f "Pressure surrounding the fuselage";
  Modelica.Units.SI.Temperature T_aw "Recovery temperature";
  Real r "Recovery factor for turbulent boundary layer";
  Modelica.Units.SI.ReynoldsNumber Re_star(start=1e8)
    "Reynolds number used in the heat transfer correlation";
  Modelica.Units.SI.PrandtlNumber Pr_star
    "Prandtl number used in the heat transfer correlation";

protected
  Medium.ThermodynamicState state_aw;
  Medium.ThermodynamicState state_star;
  Real gamma_amb;
  Modelica.Units.SI.PrandtlNumber Pr_aw(start=0.75);
  Modelica.Units.SI.Temperature T_star;

equation
  state_aw = Medium.setState_pTX(P_f, T_aw, environment.X_amb);
  state_star = Medium.setState_pTX(P_f, T_star, environment.X_amb);
  gamma_amb = Medium.isentropicExponent(environment.state_amb);
  V_inf = environment.Mach_inf*sqrt(gamma_amb*environment.R*environment.T_amb);
  P_f = environment.P_amb + Cp*Medium.density(environment.state_amb)*V_inf^2/2;
  T_aw = environment.T_amb*(1 + r*(gamma_amb - 1)/2*environment.Mach_inf^2);
  T_star = (T_aw + environment.T_amb)/2 + 0.22*(T_aw - environment.T_amb);
  Pr_aw = Medium.specificHeatCapacityCp(state_aw)*Medium.dynamicViscosity(state_aw)/
    Medium.thermalConductivity(state_aw);
  r = Pr_aw^(1/3);
  Re_star = Medium.density(state_star)*V_inf*L_nose/Medium.dynamicViscosity(state_star);
  Pr_star = Medium.specificHeatCapacityCp(state_star)*Medium.dynamicViscosity(state_star)/
    Medium.thermalConductivity(state_star);
  ht = Medium.density(state_star)*Medium.specificHeatCapacityCp(state_star)*
    V_inf*0.185*((Modelica.Math.log10(Re_star))^(-2.584))*(Pr_star^(-2/3));
  T_out = T_aw;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><b>Reference:</b></p>
<p>[1] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;HVAC&nbsp;Applications,&nbsp;chapter&nbsp;13</p>
</html>"));
end AircraftFlying;
