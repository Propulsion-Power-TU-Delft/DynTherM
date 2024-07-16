within DynTherM.Utilities.PumpCharacteristics.Flow;
class fixed "Head fixed at nominal value"
  extends BaseClass;

  input SpecificEnergy Head_nom "Nominal head provided by the pump";
  SpecificEnergy Head "Head provided by the pump";

equation
  Head = Head_nom;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end fixed;
