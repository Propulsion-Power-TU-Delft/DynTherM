within DynTherM.Utilities.PumpCharacteristics;
class BaseClass
  input AngularVelocity omega_nom "Nominal rotational speed";
  input AngularVelocity omega "Rotational speed";
  input VolumeFlowRate volFlow_nom "Nominal volumetric flow rate";
  input VolumeFlowRate volFlow "Volumetric flow rate";

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClass;
