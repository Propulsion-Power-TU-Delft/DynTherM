within DynTherM.Materials;
model Aluminium7075T6 "Aluminium alloy 7075T6"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2795.7,
    lambda=130,
    cm=960);
  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] R. Roy, et al. &quot;Multiphysics anti-icing simulation of a CFRP composite wing structure embedded with thin etched-foil electrothermal heating films in glaze ice conditions&quot;, Composite Structures, 2021.</p>
</html>"));
end Aluminium7075T6;
