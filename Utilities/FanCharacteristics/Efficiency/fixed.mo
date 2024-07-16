within DynTherM.Utilities.FanCharacteristics.Efficiency;
class fixed "Efficiency fixed at nominal value"
  extends BaseClass;

  input Real eta_nom "Nominal isentropic efficiency of the fan";
  Real eta "Isentropic efficiency of the fan";

equation
  eta = eta_nom;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end fixed;
