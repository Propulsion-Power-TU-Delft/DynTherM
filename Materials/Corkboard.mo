within DynTherM.Materials;
model Corkboard "Corkboard used for thermal insulation"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=130,
    lambda=0.043,
    cm=1900);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] R. Roy, et al. &quot;Multiphysics anti-icing simulation of a CFRP composite wing structure embedded with thin etched-foil electrothermal heating films in glaze ice conditions&quot;, Composite Structures, 2021.</p>
</html>"));
end Corkboard;
