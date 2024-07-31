within DynTherM.Components.MassTransfer.DPCorrelations;
model Idelchik_Correlation
  "Friction factor for smoothly curved tubes and channels"
  input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));
  input Length R_bend "Radius of curvature of the bend" annotation(Dialog(enable = true));
  input Angle theeta "Angle of bend" annotation(Dialog(enable = true));

  Real K_friction "Loss coefficient due to friction";
  Real lamda "Coefficient dependent of Re and bend to diameter ratio";
  Real Rr "Bend ratio";


equation

  Rr = Dh/(2*R_bend);
  K_friction = 0.0175 * lamda * theeta * R_bend/Dh;

  if Re*sqrt(Rr) < 600 then
    lamda =  (20/Re^(0.65)) * Rr^0.175;
  elseif Re*sqrt(Rr) >= 600 and Re*sqrt(Rr) <= 1400 then
    lamda = (10.4/Re^0.55) * Rr^0.225;
  else
    lamda = (5/Re^0.45) * Re^0.275;
  end if;




  // Sanity check
  assert(not
            ((Re*sqrt(Rr) > 5e3) and (Re*sqrt(Rr) < 50)), "The relation is not applicable for such conditions", AssertionLevel.warning);

end Idelchik_Correlation;
