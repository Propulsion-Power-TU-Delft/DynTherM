within DynTherM.Components.MassTransfer.DPCorrelations;
model DarcyWeisbach "Friction factor for circular pipes"
  extends BaseClass;
  parameter Length ks "Surface roughness";
  input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));

equation
  if Re < 2000 then                    // Laminar
    f = 64/Re;
  elseif Re > 4000 and Re < 10^4 then  // Transition
    f = 0.316/regPow(Re, 0.25, 1e-6);
  else                                 // Turbulent
    1/regRoot(f, 1e-6) = -2*log10(ks/(3.7*Dh) + 2.51/(Re*regRoot(f, 1e-6)));
  end if;

  annotation (Documentation(info="<html>
<p><b>Reference:</b></p>
<p>[1] C. F. Colebrook. &quot;Turbulent Flow in Pipes, with Particular Reference to the Transition Region between the Smooth and Rough Pipe Laws&quot;. J. Inst. Civil Eng., Vol. 11, 133, 1938.</p>
</html>"));
end DarcyWeisbach;
