within DynTherM.Components.MassTransfer.FanCharacteristics;
class BaseClass
  input AngularVelocity omega_nom
    "Nominal rotational speed";
  input AngularVelocity omega "Rotational speed";
  input VolumeFlowRate volFlow_nom
    "Nominal volumetric flow rate";
  input VolumeFlowRate volFlow "Volumetric flow rate";
  input SpecificEnergy Head_nom
    "Nominal head provided by the fan";
  SpecificEnergy Head "Head provided by the fan";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClass;
