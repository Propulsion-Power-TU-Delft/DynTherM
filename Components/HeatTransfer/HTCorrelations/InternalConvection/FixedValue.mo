within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model FixedValue
  "Fixed heat transfer coefficient"
  extends BaseClassInternal;
  parameter CoefficientOfHeatTransfer ht_fixed[Nx,Ny]=ones(Nx,Ny) "Heat transfer coefficient - fixed value";

equation
  ht = ht_fixed;

end FixedValue;
