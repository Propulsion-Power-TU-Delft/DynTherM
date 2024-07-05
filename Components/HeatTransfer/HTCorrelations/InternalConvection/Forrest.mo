within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model Forrest
  "Internal convection for rectangular mini-channels"
  extends BaseClassInternal;

  input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input Real phi_star "Geometrical correction" annotation(Dialog(enable = true));
  input ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));
  input PrandtlNumber Pr "Prandtl number" annotation(Dialog(enable = true));
  input Medium.ThermodynamicState state "Average thermodynamic state" annotation(Dialog(enable = true));

  NusseltNumber Nu(start=10) "Nusselt number";

equation

  // Nusselt number
  Nu = 0.199*regPow(Re - 600, 7/8, 1e-6)*Pr/(5*(Pr - 2)*phi_star^(1/8) + 10.05*regPow(Re - 600, 1/8, 1e-6)*phi_star^(1/4));
  Nu = ht*Dh/Medium.thermalConductivity(state);

  // Sanity check
  assert(Re >= 4e3, "The Forrest correlation is strictly valid for Reynolds numbers greater than 4e3", AssertionLevel.warning);
  assert(Re <= 70e3, "The Forrest correlation is strictly valid for Reynolds numbers lower than 70e3", AssertionLevel.warning);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Range of validity: 4.000 &lt; Re &lt; 70.000</p>
<p><br><b>Reference: </b></p>
<p>[1] E.C. Forrest et al. &quot;Convective Heat Transfer in a High Aspect Ratio Minichannel Heated on One Side&quot;, Journal of Heat transfer, 2016.</p>
</html>"));
end Forrest;
