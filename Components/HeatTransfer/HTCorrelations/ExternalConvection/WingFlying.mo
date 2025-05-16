within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model WingFlying
  "Forced convection along a wing section placed in the slipstream of a propeller (distributed propulsion) at cruise conditions"
  extends BaseClasses.BaseClassForcedConvectionExternal;

equation
  Nu = 0.385*(Re^0.755)*(Pr^5.285)*(environment.T_amb/T_surf)^0.044;
  T_out = environment.T_amb;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Correlation derived from empirical fitting of CFD simulations.</p>
<p><br><b>Reference:</b></p>
<p>[1] A. L. Habermann, et al. &quot;Aerodynamic Effects of a Wing Surface Heat Exchanger&quot;, MDPI Aerospace, 2023.</p>
</html>"));
end WingFlying;
