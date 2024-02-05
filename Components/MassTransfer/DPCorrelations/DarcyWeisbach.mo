within DynTherM.Components.MassTransfer.DPCorrelations;
model DarcyWeisbach "Friction factor for circular pipes according to Darcy-Weisbach correlation"
  extends BaseClass;
  parameter Length Roughness "Pipe roughness";
  input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input ReynoldsNumber Re[Nx,Ny] "Reynolds number" annotation(Dialog(enable = true));

equation
  for i in 1:Nx loop
    for j in 1:Ny loop
      if Re[i,j] < 2000 then                         // Laminar
        f[i,j] = 64/Re[i,j];
      elseif Re[i,j] > 4000 and Re[i,j] < 10^4 then  // Transition
        f[i,j] = 0.316/(Re[i,j]^0.25);
      else                                           // Turbulent
        1/sqrt(f[i,j]) = -2*log10(Roughness/(3.7*Dh) + 2.51/(Re[i,j]*sqrt(f[i,j])));
      end if;
    end for;
  end for;

end DarcyWeisbach;
