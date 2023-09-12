within ThermalManagement.Components.MassTransfer.FanCharacteristics.FlowCharacteristics;
class dummyFlow "Head fixed at nominal value"
  extends BaseClass;
  Modelica.Units.SI.SpecificEnergy Head "Head provided by the fan";
equation
  Head = Head_nom;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end dummyFlow;
