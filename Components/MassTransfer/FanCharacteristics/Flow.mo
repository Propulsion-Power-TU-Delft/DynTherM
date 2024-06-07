within DynTherM.Components.MassTransfer.FanCharacteristics;
package Flow

  class fixed "Head fixed at nominal value"
    extends BaseClass;

    input SpecificEnergy Head_nom "Nominal head provided by the fan";
    SpecificEnergy Head "Head provided by the fan";

  equation
    Head = Head_nom;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end fixed;

  class linear "Linear flow characteristic"
    extends BaseClass;

    input SpecificEnergy Head_nom "Nominal head provided by the fan";
    SpecificEnergy Head "Head provided by the fan";
    VolumeFlowRate volFlow_max "Maximum volumetric flow rate processed by the fan";

  protected
    Real c1;
    Real c2;

  equation
    volFlow_max = volFlow_nom*2;  // Hp: the nominal operating point is in the middle of the performance map
    Head_nom = c1 + volFlow_nom*c2;
    c1 + volFlow_max*c2 = 0;
    Head = (omega/omega_nom)^2*(c1 + volFlow*c2);
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end linear;
end Flow;
