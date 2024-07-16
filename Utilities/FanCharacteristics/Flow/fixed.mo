within DynTherM.Utilities.FanCharacteristics.Flow;
class fixed "Head fixed at nominal value"
  extends BaseClass;

  input SpecificEnergy Head_nom "Nominal head provided by the fan";
  SpecificEnergy Head "Head provided by the fan";

equation
  Head = Head_nom;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end fixed;
