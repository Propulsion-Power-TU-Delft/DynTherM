within ThermalManagement.Materials;
model Opticor "Transparent plastic material used for aircraft glazing"
  // References:
  // [1] Opticor, Advanced Transparencies Material - Technical Data Brochure.
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=1130,
    lambda=0.21,
    cm=840,
    n=1.52,
    t=0.9);
end Opticor;
