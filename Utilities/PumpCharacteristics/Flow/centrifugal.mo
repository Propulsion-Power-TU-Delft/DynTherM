within DynTherM.Utilities.PumpCharacteristics.Flow;
class centrifugal "Centrifugal pump flow characteristic"
  extends BaseClass;

  input SpecificEnergy Head_nom "Nominal head provided by the pump";
  parameter Real c[2] = {2.462215552,  -0.53791904}
    "Default coefficients for head vs. volumetric flow curve";

  SpecificEnergy Head "Head provided by the pump";

equation
  Head = Head_nom*(c[1] + c[2]*exp(volFlow/volFlow_nom))*(omega/omega_nom)^2;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end centrifugal;
