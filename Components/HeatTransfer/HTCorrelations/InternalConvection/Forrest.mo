within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model Forrest
  "Internal convection for rectangular mini-channels according to correlation of Forrest et al."
  extends BaseClassInternal;
  input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input Real phi_star "Geometrical correction" annotation(Dialog(enable = true));
  input ReynoldsNumber Re[Nx,Ny] "Reynolds number" annotation(Dialog(enable = true));
  input PrandtlNumber Pr[Nx,Ny] "Prandtl number" annotation(Dialog(enable = true));
  input Medium.ThermodynamicState state[Nx,Ny] "Average thermodynamic state" annotation(Dialog(enable = true));

  NusseltNumber Nu[Nx,Ny] "Nusselt number";

equation

  for i in 1:Nx loop
    for j in 1:Ny loop
      // Nusselt number
      Nu[i,j] = 0.199*(Re[i,j] - 600)^(7/8)*Pr[i,j]/(5*(Pr[i,j] - 2)*phi_star^(1/8) +
        10.05*(Re[i,j] - 600)^(1/8)*phi_star^(1/4));
      Nu[i,j] = ht[i,j]*Dh/Medium.thermalConductivity(state[i,j]);

      // Sanity check
      assert(Re[i,j] >= 4e3, "The Forrest correlation is valid only for
        Reynolds numbers greater than 4e3", AssertionLevel.warning);
      assert(Re[i,j] <= 70e3, "The Forrest correlation is valid only for
        Reynolds numbers lower than 70e3", AssertionLevel.warning);
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Reference: E.C. Forrest et al. &quot;Convective Heat Transfer in a High Aspect Ratio Minichannel Heated on One Side&quot;, Journal of Heat transfer, 2016.</p>
<p>Range of validity: 4.000 &lt; Re &lt; 70.000</p>
</html>"));
end Forrest;
