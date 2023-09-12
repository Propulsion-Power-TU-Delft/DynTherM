within DynTherM.Materials;
model GlassPhenolic "Glass phenolic"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2550,
    lambda=0.24,
    cm=1110);
end GlassPhenolic;
