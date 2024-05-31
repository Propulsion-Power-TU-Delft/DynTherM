within DynTherM.Components.MassTransfer.FanCharacteristics.FlowCharacteristics;
class dummyFlow "Head fixed at nominal value"
  extends BaseClass;
  SpecificEnergy Head "Head provided by the fan";
equation
  Head = Head_nom;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end dummyFlow;
