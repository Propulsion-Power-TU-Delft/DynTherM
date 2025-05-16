within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model FixedValue "Fixed ht value"
  extends BaseClasses.BaseClassExternal;
  parameter CoefficientOfHeatTransfer ht_fixed=1 "Heat transfer coefficient - fixed value";

equation
  ht = ht_fixed;
  T_out = environment.T_amb;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FixedValue;
