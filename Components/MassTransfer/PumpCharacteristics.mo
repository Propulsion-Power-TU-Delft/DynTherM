within DynTherM.Components.MassTransfer;
package PumpCharacteristics
  "Package collecting different laws describing the characteristic curves of pumps"

  class BaseClass
    input AngularVelocity omega_nom "Nominal rotational speed";
    input AngularVelocity omega "Rotational speed";
    input VolumeFlowRate volFlow_nom "Nominal volumetric flow rate";
    input VolumeFlowRate volFlow "Volumetric flow rate";

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end BaseClass;

  package Flow

    class fixed "Head fixed at nominal value"
      extends BaseClass;

      input SpecificEnergy Head_nom "Nominal head provided by the pump";
      SpecificEnergy Head "Head provided by the pump";

    equation
      Head = Head_nom;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end fixed;

    class centrifugal "Centrifugal pump flow characteristic"
      extends BaseClass;

      input SpecificEnergy Head_nom "Nominal head provided by the pump";
      parameter Real c[2] = {2.462215552,  -0.53791904}
        "Default coefficients for head vs. volumetric flow curve";

      SpecificEnergy Head "Head provided by the pump";

    equation
      Head = Head_nom*(c[1] + c[2]*exp(volFlow/volFlow_nom))*(omega/omega_nom)^2;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end centrifugal;
  end Flow;

  package Efficiency

    class fixed "Efficiency fixed at nominal value"
      extends BaseClass;

      input Real eta_nom "Nominal isentropic efficiency of the pump";
      Real eta "Isentropic efficiency of the pump";

    equation
      eta = eta_nom;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end fixed;

    class Veres "Efficiency computed with Veres model"
      extends BaseClass;

      input Real eta_nom "Nominal isentropic efficiency of the pump";
      parameter Real a[4] = {-0.029265, -0.14086, 0.3096, 0.860525} "Veres coefficients";

      Real eta "Isentropic efficiency of the pump";
      Real F "Volumetric flow ratio";

    equation
      F = volFlow/volFlow_nom*omega_nom/omega;
      eta = eta_nom*(a[1]*F^3 + a[2]*F^2 + a[3]*F + a[4]);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<p><b>Reference</b>:</p>
<p>[1] J. P. Veres. &quot;Centrifugal and Axial Pump Design and Off-Design Performance Prediction&quot;, NASA Technical Memorandum, 1994.</p>
</html>"));
    end Veres;
  end Efficiency;
  annotation (Icon(graphics={
        Line(
          points={{-80,-80},{-80,100}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{-80,-80},{100,-80}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{-80,100},{-94,80}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{-80,100},{-66,80}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{100,-80},{80,-66}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{100,-80},{80,-94}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{20,-80},{20,4},{-80,4}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{-80,60},{0,28},{36,-16},{68,-80}},
          color={0,0,0},
          smooth=Smooth.Bezier),
        Ellipse(
          extent={{16,8},{24,0}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid)}));
end PumpCharacteristics;
