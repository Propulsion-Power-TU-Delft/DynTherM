within DynTherM.Choices;
type PDropOpt = enumeration(
    fixed "Constant pressure drop",
    linear "Linear pressure drop computed with hydraulic resistance",
    correlation "Pressure drop calculated with semi-empirical correlation")
  "Menu choices to select the PressureDrop model";
