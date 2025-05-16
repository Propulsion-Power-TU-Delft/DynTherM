within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model WingTakeOff
  "Forced convection along a wing section placed in the slipstream of a propeller (distributed propulsion) at take-off conditions"
  extends BaseClasses.BaseClassForcedConvectionExternal;

equation
  Nu = 0.176*(Re^0.827)*(Pr^6.453)*(environment.T_amb/T_surf)^(-0.022);
  T_out = environment.T_amb;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Correlation derived from empirical fitting of CFD simulations.</p>
<p><br><b>Reference:</b></p>
<p>[1] A. L. Habermann, et al. &quot;Aerodynamic Effects of a Wing Surface Heat Exchanger&quot;, MDPI Aerospace, 2023.</p>
</html>"));
end WingTakeOff;
