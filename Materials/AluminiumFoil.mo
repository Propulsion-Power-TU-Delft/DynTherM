within DynTherM.Materials;
model AluminiumFoil "Aluminium foil 1100-H19"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2700,
    lambda=218,
    cm=904);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] R. Roy, et al. &quot;Multiphysics anti-icing simulation of a CFRP composite wing structure embedded with thin etched-foil electrothermal heating films in glaze ice conditions&quot;, Composite Structures, 2021.</p>
</html>"));
end AluminiumFoil;
