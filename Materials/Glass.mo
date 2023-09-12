within ThermalManagement.Materials;
model Glass "Common glass"
  // References:
  // [1] J. Deng et al. - Study on the thermodynamic characteristic matching property and limit
  // design principle of general flat plate solar air collectors (FPSACs) - Building Simulation, 2016.
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2500,
    lambda=0.8,
    cm=800,
    n=1.526,
    t=0.839);
end Glass;
