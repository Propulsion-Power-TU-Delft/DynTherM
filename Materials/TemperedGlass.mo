within DynTherM.Materials;
model TemperedGlass "Tempered glass"
  // References:
  // [1] J. Deng et al. - Study on the thermodynamic characteristic matching property and limit
  // design principle of general flat plate solar air collectors (FPSACs) - Building Simulation, 2016.
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2500,
    lambda=0.8,
    cm=800,
    n=1.47,
    t=0.918);
end TemperedGlass;
