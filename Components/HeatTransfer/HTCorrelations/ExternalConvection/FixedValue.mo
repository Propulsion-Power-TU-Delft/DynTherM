within ThermalManagement.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model FixedValue "Fixed ht value"
  extends BaseClassExternal;
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_fixed
    "Heat transfer coefficient - fixed value";
equation
  ht = ht_fixed;
  T_out = environment.T_amb;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FixedValue;
