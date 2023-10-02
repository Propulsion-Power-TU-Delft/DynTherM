within DynTherM.Components.MassTransfer.DPCorrelations;
model DarcyWeisbach "Friction factor for circular pipes according to Darcy-Weisbach correlation"
  extends BaseClass;
  parameter Length Roughness "Pipe roughness";
  input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));

equation
  if Re < 2000 then                    // Laminar
    f = 64/Re;
  elseif Re > 4000 and Re < 10^4 then  // Transition
    f = 0.316/(Re^0.25);
  else                                 // Turbulent
    1/sqrt(f) = -2*log10(Roughness/(3.7*Dh) + 2.51/(Re*sqrt(f)));
  end if;

end DarcyWeisbach;
