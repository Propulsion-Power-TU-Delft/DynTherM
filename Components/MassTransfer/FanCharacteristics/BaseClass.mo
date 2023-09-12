within ThermalManagement.Components.MassTransfer.FanCharacteristics;
class BaseClass
  input Modelica.Units.SI.AngularVelocity omega_nom
    "Nominal rotational speed";
  input Modelica.Units.SI.AngularVelocity omega "Rotational speed";
  input Modelica.Units.SI.VolumeFlowRate volFlow_nom
    "Nominal volumetric flow rate";
  input Modelica.Units.SI.VolumeFlowRate volFlow "Volumetric flow rate";
  input Modelica.Units.SI.SpecificEnergy Head_nom
    "Nominal head provided by the fan";
  Modelica.Units.SI.SpecificEnergy Head "Head provided by the fan";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseClass;
