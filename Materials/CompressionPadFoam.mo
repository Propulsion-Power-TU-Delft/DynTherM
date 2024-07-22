within DynTherM.Materials;
model CompressionPadFoam
  "Compressible foam used as compression pad in between battery cells"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=300,
    lambda=0.1,
    cm=1000);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] I. Gul. &quot;Time efficient simulations for advanced battery cooling concepts&quot;, M.Sc. thesis, Polestar / Universitat Politecnica de Catalunya, 2023.</p>
</html>"));
end CompressionPadFoam;
