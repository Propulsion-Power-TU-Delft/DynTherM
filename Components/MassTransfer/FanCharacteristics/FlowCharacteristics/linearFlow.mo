within ThermalManagement.Components.MassTransfer.FanCharacteristics.FlowCharacteristics;
class linearFlow "Linear flow characteristic"
  extends BaseClass;
  Modelica.Units.SI.SpecificEnergy Head "Head provided by the fan";
protected
  Modelica.Units.SI.VolumeFlowRate volFlow_max
    "Maximum volumetric flow rate processed by the fan";
  Real c1;
  Real c2;
equation
  volFlow_max = volFlow_nom*2;  // Hp: the nominal operating point is in the middle of the performance map
  Head_nom = c1 + volFlow_nom*c2;
  c1 + volFlow_max*c2 = 0;
  Head = (omega/omega_nom)^2*(c1 + volFlow*c2);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end linearFlow;
