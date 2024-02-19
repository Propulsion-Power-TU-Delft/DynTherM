within DynTherM.Materials;
model NFRSB31 "Polyurethane foam"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=31,
    lambda=0.049,
    cm=2996);
  annotation (Documentation(info="<html>
<p><b>Reference:</b></p>
<p>[1] D. S. W. Pau, et al. &quot;Thermophysical Properties of Polyurethane Foams and their Melts&quot;, Fire and Materials, 2013.</p>
</html>"));
end NFRSB31;
