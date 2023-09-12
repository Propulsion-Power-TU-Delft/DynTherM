within DynTherM.Materials;
model Polycarbonate "Polycarbonate"
  // References:
  // [1] J. Deng et al. - Study on the thermodynamic characteristic matching property and limit
  // design principle of general flat plate solar air collectors (FPSACs) - Building Simulation, 2016.
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=1210,
    lambda=0.2,
    cm=1250,
    n=1.6,
    t=0.79);
end Polycarbonate;
