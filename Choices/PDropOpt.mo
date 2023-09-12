within DynTherM.Choices;
type PDropOpt = enumeration(
    fixed "Constant pressure drop",
    linear "Linear pressure drop computed with hydraulic resistance",
    darcyWeisbach "Pressure drop calculated with Darcy-Weisbach relation")
  "Menu choices to select the PressureDrop model";
