within DynTherM.Materials;
model GlassFibre "Glass fibre used for insulation blankets"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=9.6,
    lambda=0.036,
    cm=1005);
end GlassFibre;
