within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model Wing
  "Forced convection along a wing section placed in the slipstream of a propeller (distributed propulsion) - blend between take-off and cruise conditions"
  extends BaseClasses.BaseClassForcedConvectionExternal;
  parameter Length altitude_transition=3000 "The correlation for take-off is blended with the one for cruise up to this value of altitude";

  NusseltNumber Nu_takeoff "Nusselt number at take-off conditions";
  NusseltNumber Nu_cruise "Nusselt number at cruise conditions";

equation
  T_out = environment.T_amb;
  Nu_takeoff = 0.176*(Re^0.827)*(Pr^6.453)*(environment.T_amb/T_surf)^(-0.022);
  Nu_cruise = 0.385*(Re^0.755)*(Pr^5.285)*(environment.T_amb/T_surf)^0.044;

  if environment.altitude < altitude_transition then
    Nu = Nu_takeoff*(1 - environment.altitude/altitude_transition) +
         Nu_cruise*environment.altitude/altitude_transition;
  else
    Nu = Nu_cruise;
  end if;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Correlation derived from empirical fitting of CFD simulations.</p>
<p><br><b>Reference:</b></p>
<p>[1] A. L. Habermann, et al. &quot;Aerodynamic Effects of a Wing Surface Heat Exchanger&quot;, MDPI Aerospace, 2023.</p>
</html>"));
end Wing;
