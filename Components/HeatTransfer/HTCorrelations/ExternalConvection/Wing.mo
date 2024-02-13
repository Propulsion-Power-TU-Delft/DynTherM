within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model Wing
  "Forced convection along a wing section placed in the slipstream of a propeller (distributed propulsion) - blend between take-off and cruise conditions"
  extends BaseClassExternal;

  input Length c "Airfoil chord" annotation(Dialog(enable=true));
  parameter Length altitude_transition=3000 "The correlation for take-off is blended with the one for cruise up to this value of altitude";

  PrandtlNumber Pr "Prandtl number";
  ReynoldsNumber Re "Reynolds number";
  NusseltNumber Nu_takeoff "Nusselt number computed with correlation for take-off condition";
  NusseltNumber Nu_cruise "Nusselt number computed with correlation for cruise condition";
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

  Nu_takeoff = 0.176*Re^0.827*Pr^6.453*(environment.T_amb/T_skin)^(-0.022);
  Nu_cruise = 0.385*Re^0.755*Pr^5.285*(environment.T_amb/T_skin)^0.044;

  if environment.altitude < altitude_transition then
    Nu = Nu_takeoff*(1 - environment.altitude/altitude_transition) +
         Nu_cruise*environment.altitude/altitude_transition;
  else
    Nu = Nu_cruise;
  end if;

  Nu = ht*c/Medium.thermalConductivity(state_f);
  T_out = environment.T_amb;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Correlation derived from empirical fitting of CFD simulations.</p>
<p><br><b>Reference:</b></p>
<p>[1] A. L. Habermann, et al. &quot;Aerodynamic Effects of a Wing Surface Heat Exchanger&quot;, MDPI Aerospace, 2023.</p>
</html>"));
end Wing;
