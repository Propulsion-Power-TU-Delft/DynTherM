within DynTherM.Choices;
type ConvectionOpt = enumeration(
    fixedValue "Fixed heat transfer coefficient",
    groundFree "Ground condition, free convection",
    groundForced "Ground condition, forced convection",
    flying "Flying condition")
  "Menu choices to select the correlation used in model ExternalConvection";
