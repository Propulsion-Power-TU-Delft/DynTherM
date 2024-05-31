within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model Cylinder
  "Internal convection for cylindrical shape"
  extends BaseClassInternal;
  parameter Velocity V_int=1 "Air speed inside plenum";

equation
  ht = 5.6783*(2 + 0.314*3.2808*V_int);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><b>Reference:</b></p>
<p>[1] SAE Aerospace. &quot;AIR 1168/3 - Aerothermodynamic Systems Engineering and Design&quot;, 2011.</p>
</html>"));
end Cylinder;
