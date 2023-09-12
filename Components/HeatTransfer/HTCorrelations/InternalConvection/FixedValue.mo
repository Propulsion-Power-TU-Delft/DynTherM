within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model FixedValue
  "Fixed heat transfer coefficient"
  extends BaseClassInternal;
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_fixed
    "Heat transfer coefficient - fixed value";
equation
  ht = ht_fixed;
end FixedValue;
