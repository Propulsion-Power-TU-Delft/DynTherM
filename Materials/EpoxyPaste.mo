within DynTherM.Materials;
model EpoxyPaste "Epoxy adhesive paste"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=1360,
    lambda=0.33,
    cm=1000);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] R. Roy, et al. &quot;Multiphysics anti-icing simulation of a CFRP composite wing structure embedded with thin etched-foil electrothermal heating films in glaze ice conditions&quot;, Composite Structures, 2021.</p>
</html>"));
end EpoxyPaste;
