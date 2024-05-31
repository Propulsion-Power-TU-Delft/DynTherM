within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model DittusBoelter "Internal convection for circular pipes"
  extends BaseClassInternal;

  input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input Temperature T_in "Inlet fluid temperature" annotation(Dialog(enable = true));
  input Temperature T_out "Outlet fluid temperature" annotation(Dialog(enable = true));
  input ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));
  input PrandtlNumber Pr "Prandtl number" annotation(Dialog(enable = true));
  input Medium.ThermodynamicState state "Average thermodynamic state" annotation(Dialog(enable = true));

  NusseltNumber Nu "Nusselt number";

equation

  // Cooled pipe
  if T_in >= T_out then
    Nu = 0.023*Re^(4/5)*Pr^0.3;
  else
  // Heated pipe
    Nu = 0.023*Re^(4/5)*Pr^0.4;
  end if;

  Nu = ht*Dh/Medium.thermalConductivity(state);

  // Sanity check
  assert(Re >= 10e3, "The Dittus-Boelter correlation is valid only for Reynolds numbers greater than 10e3", AssertionLevel.warning);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><b>Reference:</b></p>
<p>[1] &quot;ASHRAE Handbook - Fundamentals&quot;, 2013.</p>
</html>"));
end DittusBoelter;
