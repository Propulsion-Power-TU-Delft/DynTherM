within DynTherM.Materials;
model AluminiumColdPlate
  "Aluminium used for cold plate in battery pack applications"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2560,
    lambda=92,
    cm=963);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] I. Gul. &quot;Time efficient simulations for advanced battery cooling concepts&quot;, M.Sc. thesis, Polestar / Universitat Politecnica de Catalunya, 2023.</p>
</html>"));
end AluminiumColdPlate;
