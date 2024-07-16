within DynTherM.Utilities.PumpCharacteristics.Efficiency;
class fixed "Efficiency fixed at nominal value"
  extends BaseClass;

  input Real eta_nom "Nominal isentropic efficiency of the pump";
  Real eta "Isentropic efficiency of the pump";

equation
  eta = eta_nom;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end fixed;
