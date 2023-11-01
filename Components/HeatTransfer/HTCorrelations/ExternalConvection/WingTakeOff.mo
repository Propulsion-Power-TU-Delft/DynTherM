within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model WingTakeOff
  "Forced convection along a wing section placed in the slipstream of a propeller (distributed propulsion) at take-off conditions"
  extends BaseClassExternal;

  input Length c "Airfoil chord";

  PrandtlNumber Pr "Prandtl number";
  ReynoldsNumber Re "Reynolds number";
  NusseltNumber Nu "Nusselt number";
  Velocity V_inf "Aircraft velocity";

protected
  Medium.ThermodynamicState state_f;
  Temperature Tf "Film temperature";
  Real gamma_amb;

equation
  state_f = Medium.setState_pTX(environment.P_amb, Tf, environment.X_amb);
  Tf = (T_skin + environment.T_amb)/2;
  V_inf = environment.Mach_inf*sqrt(gamma_amb*environment.R*environment.T_amb);
  gamma_amb = Medium.isentropicExponent(environment.state_amb);

  Pr = Medium.specificHeatCapacityCp(state_f)*Medium.dynamicViscosity(state_f)/
    Medium.thermalConductivity(state_f);
  Re = Medium.density(state_f)*V_inf*c/Medium.dynamicViscosity(state_f);
  Nu = 0.176*Re^0.827*Pr^6.453*(environment.T_amb/T_skin)^(-0.022);
  Nu = ht*c/Medium.thermalConductivity(state_f);
  T_out = environment.T_amb;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Reference:</p>
<p>[1] A. L. Habermann, et al. &quot;Aerodynamic Effects of a Wing Surface Heat Exchanger&quot;, MDPI Aerospace, 2023.</p>
</html>"));
end WingTakeOff;
