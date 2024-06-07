within DynTherM.Materials;
model PolestarCell "Material used for battery cells of Polestar"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2740,
    lambda=20,
    cm=1204);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] R. Roy, et al. &quot;Multiphysics anti-icing simulation of a CFRP composite wing structure embedded with thin etched-foil electrothermal heating films in glaze ice conditions&quot;, Composite Structures, 2021.</p>
</html>"));
end PolestarCell;
