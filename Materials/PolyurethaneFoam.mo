within DynTherM.Materials;
model PolyurethaneFoam
  "Compressible polyurethane foam used as firewall material in between battery cells"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=204.1,
    lambda=0.0726,
    cm=1604);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] C. Yang, et al. &quot;Compressible battery foams to prevent cascading thermal runaway in Li-ion pouch batteries&quot;, Journal of Power Sources, 2022.</p>
</html>"));
end PolyurethaneFoam;
