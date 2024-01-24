within DynTherM.Materials;
model CarbonEpoxyLamina
  "USN-125B carbon/epoxy prepreg unidirectional lamina"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=1550,
    lambda=0.95,
    cm=929);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] R. Roy, et al. &quot;Multiphysics anti-icing simulation of a CFRP composite wing structure embedded with thin etched-foil electrothermal heating films in glaze ice conditions&quot;, Composite Structures, 2021.</p>
</html>"));
end CarbonEpoxyLamina;
