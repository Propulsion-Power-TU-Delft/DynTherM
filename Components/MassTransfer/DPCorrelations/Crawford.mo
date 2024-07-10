within DynTherM.Components.MassTransfer.DPCorrelations;
model Crawford "Friction factor for pipe elbows with circular cross-section"
  extends BaseClass;
  parameter Length Roughness "Pipe roughness";
  input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));
  input Length R_bend "Radius of curvature of the elbow" annotation(Dialog(enable = true));

  Real Rr "Elbow radius ratio";
  Real Kt(start=1.2) "Total loss coefficient";
  Real K_friction "Loss coefficient due to friction";
  Real K_separation "Loss coefficient due to flow separation";

equation
  Rr = R_bend/(2*Dh);

  // Darcy-Weisbach for straight circular pipes
  if Re < 2000 then                    // Laminar
    f = 64/Re;
  elseif Re > 4000 and Re < 10^4 then  // Transition
    f = 0.316/regPow(Re, 0.25, 1e-6);
  else                                 // Turbulent
    1/regRoot(f, 1e-6) = -2*log10(Roughness/(3.7*Dh) + 2.51/(Re*regRoot(f, 1e-6)));
  end if;

  K_friction = 2.4792*Rr*regPow(Re, 0.25, 1e-6)*4*f;
  K_separation = 1.25*Rr^(-1.5)*regPow(Re, 0.35, 1e-6)*4*f;
  Kt = K_friction + K_separation;

  // Sanity check
  assert(not
            ((Re > 4e3) and (Re < 3e5)), "The Crawford correlation is strictly valid for Re > 4e3 and Re < 3e5", AssertionLevel.warning);

  annotation (Documentation(info="<html>
<h4>Reference:</h4>
<p>[1] N. Crawford et al. &quot;An Experimental Investigation into the Pressure Drop for Turbulent Flow 90 Degrees Elbow Bends&quot;, Institution of Mechanical Engineers. Proceedings. Part E: Journal of Process Mechanical Engineering, 2007.</p>
</html>"));
end Crawford;
