within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model FixedValue "Fixed ht value"
  extends BaseClassExternal;
  parameter CoefficientOfHeatTransfer ht_fixed[Nx,Ny]=ones(Nx,Ny) "Heat transfer coefficient - fixed value";

equation
  ht = ht_fixed;
  T_out = environment.T_amb*ones(Nx,Ny);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FixedValue;
