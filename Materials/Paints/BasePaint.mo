within DynTherM.Materials.Paints;
model BasePaint
  extends Modelica.Icons.MaterialProperty;
  constant Real eps(min=0,max=1) "Thermal emittance";
  constant Real abs(min=0,max=1) "Solar absorptance";

  annotation (Documentation(info="<html>
<p>Reference:</p>
<p>[1] J. H. Henninger. &quot;Solar Absorbtance and Thermal Emittance of Some Common Spacecraft Thermal-Control Coatings&quot;, NASA Reference Publication 1121, 1984.</p>
</html>"));
end BasePaint;
