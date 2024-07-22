within DynTherM.Materials;
model ThermalResin
  "Thermal resin placed in between battery cells and monoframe"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=300,
    lambda=1,
    cm=770);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] I. Gul. &quot;Time efficient simulations for advanced battery cooling concepts&quot;, M.Sc. thesis, Polestar / Universitat Politecnica de Catalunya, 2023.</p>
</html>"));
end ThermalResin;
