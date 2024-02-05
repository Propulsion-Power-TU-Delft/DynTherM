within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model DittusBoelter "Internal convection for circular pipes according to Dittus-Boelter correlation"
  extends BaseClassInternal;
  input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input Temperature T_in "Inlet fluid temperature" annotation(Dialog(enable = true));
  input Temperature T_out "Outlet fluid temperature" annotation(Dialog(enable = true));
  input ReynoldsNumber Re[Nx,Ny] "Reynolds number" annotation(Dialog(enable = true));
  input PrandtlNumber Pr[Nx,Ny] "Prandtl number" annotation(Dialog(enable = true));
  input Medium.ThermodynamicState state[Nx,Ny] "Average thermodynamic state" annotation(Dialog(enable = true));

  NusseltNumber Nu[Nx,Ny] "Nusselt number";

equation

  for i in 1:Nx loop
    for j in 1:Ny loop
      // Cooled pipe
      if T_in >= T_out then
        Nu[i,j] = 0.023*Re[i,j]^(4/5)*Pr[i,j]^0.3;
      else
      // Heated pipe
        Nu[i,j] = 0.023*Re[i,j]^(4/5)*Pr[i,j]^0.4;
      end if;

      Nu[i,j] = ht[i,j]*Dh/Medium.thermalConductivity(state[i,j]);

      // Sanity check
      assert(Re[i,j] >= 10e3, "The Dittus-Boelter correlation is valid only
        for Reynolds numbers greater than 10e3", AssertionLevel.warning);
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Reference: &quot;ASHRAE Handbook - Fundamentals&quot;, 2013.</p>
</html>"));
end DittusBoelter;
