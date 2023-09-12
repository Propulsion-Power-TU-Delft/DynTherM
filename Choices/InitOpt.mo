within ThermalManagement.Choices;
type InitOpt = enumeration(
    noInit "No initial equations",
    fixedState "Fixed initial state variables",
    steadyState "Steady-state initialization")
  "Menu choices to select the initialization type used for the simulation";
