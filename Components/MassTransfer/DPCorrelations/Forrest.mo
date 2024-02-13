within DynTherM.Components.MassTransfer.DPCorrelations;
model Forrest "Friction factor for rectangular mini-channels"
  extends BaseClass;
  input Modelica.Units.SI.Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input Real phi_star "Geometrical correction" annotation(Dialog(enable = true));
  input Modelica.Units.SI.ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));

equation
  if Re < 2000 then                    // Laminar
    f = 64/(phi_star*Re);
  else                                 // Turbulent
    // Corrected Blasius equation --> smooth tube
    f = 0.3164*(phi_star*Re)^(-1/4);
  end if;

  // Sanity check
  assert(not
            ((Re > 2e3) and (Re < 4e3)), "The Forrest correlation is not strictly valid in the transition region", AssertionLevel.warning);

  annotation (Documentation(info="<html>
<p><b>Reference:</b></p>
<p>[1] E. C. Forrest, et al. &ldquo;Convective Heat Transfer in a High Aspect Ratio Minichannel Heated on One Side&rdquo;, Journal of Heat Transfer, 2016.</p>
</html>"));
end Forrest;
