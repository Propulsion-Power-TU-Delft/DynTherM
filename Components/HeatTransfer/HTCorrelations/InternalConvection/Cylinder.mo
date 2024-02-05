within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model Cylinder
  "Internal convection for cylindrical shape"
  extends BaseClassInternal;
  parameter Velocity V_int[Nx,Ny]=ones(Nx,Ny) "Air speed inside plenum";

equation
  for i in 1:Nx loop
    for j in 1:Ny loop
      ht[i,j] = 5.6783*(2 + 0.314*3.2808*V_int[i,j]);
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Reference: SAE Aerospace. &quot;AIR 1168/3 - Aerothermodynamic Systems Engineering and Design&quot;, 2011.</p>
</html>"));
end Cylinder;
