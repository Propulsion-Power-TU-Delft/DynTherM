within DynTherM.Utilities.PumpCharacteristics.Efficiency;
class Veres "Efficiency computed with Veres model"
  extends BaseClass;

  input Real eta_nom "Nominal isentropic efficiency of the pump";
  parameter Real a[4] = {-0.029265, -0.14086, 0.3096, 0.860525} "Veres coefficients";

  Real eta "Isentropic efficiency of the pump";
  Real F "Volumetric flow ratio";

equation
  F = volFlow/volFlow_nom*omega_nom/omega;
  eta = eta_nom*(a[1]*F^3 + a[2]*F^2 + a[3]*F + a[4]);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><b>Reference</b>:</p>
<p>[1] J. P. Veres. &quot;Centrifugal and Axial Pump Design and Off-Design Performance Prediction&quot;, NASA Technical Memorandum, 1994.</p>
</html>"));
end Veres;
