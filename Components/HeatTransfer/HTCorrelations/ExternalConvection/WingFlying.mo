within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model WingFlying
  "Forced convection along a wing section placed in the slipstream of a propeller (distributed propulsion) at cruise conditions"
  extends BaseClassExternal;

  input Length c "Airfoil chord" annotation(Dialog(enable=true));

  PrandtlNumber Pr "Prandtl number";
  ReynoldsNumber Re "Reynolds number";
  NusseltNumber Nu "Nusselt number";

protected
  Medium.ThermodynamicState state_f;
  Temperature Tf "Film temperature";

equation
  state_f = Medium.setState_pTX(environment.P_amb, Tf, environment.X_amb);
  Tf = (T_skin + environment.T_amb)/2;

  Pr = Medium.specificHeatCapacityCp(state_f)*Medium.dynamicViscosity(state_f)/
    Medium.thermalConductivity(state_f);
  Re = Medium.density(state_f)*environment.V_inf*c/Medium.dynamicViscosity(state_f);
  Nu = 0.385*Re^0.755*Pr^5.285*(environment.T_amb/T_skin)^0.044;
  Nu = ht*c/Medium.thermalConductivity(state_f);
  T_out = environment.T_amb;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Correlation derived from empirical fitting of CFD simulations.</p>
<p><br><b>Reference:</b></p>
<p>[1] A. L. Habermann, et al. &quot;Aerodynamic Effects of a Wing Surface Heat Exchanger&quot;, MDPI Aerospace, 2023.</p>
</html>"));
end WingFlying;
