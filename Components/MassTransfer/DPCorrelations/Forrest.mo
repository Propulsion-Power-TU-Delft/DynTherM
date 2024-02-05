within DynTherM.Components.MassTransfer.DPCorrelations;
model Forrest
  "Friction factor for rectangular mini-channels according to correlation of Forrest et al."
  extends BaseClass;
  input Modelica.Units.SI.Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input Real phi_star "Geometrical correction" annotation(Dialog(enable = true));
  input Modelica.Units.SI.ReynoldsNumber Re[Nx,Ny] "Reynolds number" annotation(Dialog(enable = true));

equation
  for i in 1:Nx loop
    for j in 1:Ny loop
      if Re[i,j] < 2000 then               // Laminar
        f[i,j] = 64/(phi_star*Re[i,j]);
      else                                 // Turbulent
        // Corrected Blasius equation --> smooth tube
        f[i,j] = 0.3164*(phi_star*Re[i,j])^(-1/4);
      end if;

      // Sanity check
      assert(not
                ((Re[i,j] > 2e3) and (Re[i,j] < 4e3)),
        "The Forrest correlation is not strictly valid in the transition region", AssertionLevel.warning);
    end for;
  end for;

  annotation (Documentation(info="<html>
<p>Reference</p>
<p>[1] E. C. Forrest, et al. &ldquo;Convective Heat Transfer in a High Aspect Ratio Minichannel Heated on One Side&rdquo;, Journal of Heat Transfer, 2016.</p>
</html>"));
end Forrest;
