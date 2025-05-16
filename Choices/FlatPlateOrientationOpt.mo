within DynTherM.Choices;
type FlatPlateOrientationOpt = enumeration(
    facingUp "Facing up (free convection)",
    facingDown "Facing down (free convection)",
    parallelFlow "Parallel to the flow direction (forced convection)",
    orthogonalFlow "Othogonal to the flow direction (forced convection)")
  "Menu choices to select the orientation of the flat plate";
