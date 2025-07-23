within DynTherM.Materials;
model Aluminium2024T3 "Aluminium alloy 2024-T3"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2780,
    lambda=121,
    cm=875);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] R. Roy, et al. &quot;Multiphysics anti-icing simulation of a CFRP composite wing structure embedded with thin etched-foil electrothermal heating films in glaze ice conditions&quot;, Composite Structures, 2021.</p>
</html>"));
end Aluminium2024T3;
