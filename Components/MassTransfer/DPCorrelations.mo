within DynTherM.Components.MassTransfer;
package DPCorrelations
  "Package collecting correlations for the estimation of the pressure drop"

  partial model BaseClass
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
    parameter Real f_start=0.03 "Friction factor - starting value";

    Real f(start=f_start) "Friction factor";
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end BaseClass;

  model DarcyWeisbach "Friction factor for circular pipes according to Darcy-Weisbach correlation"
    extends BaseClass;
    parameter Length Roughness "Pipe roughness";
    input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
    input ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));

  equation
    if Re < 2000 then                    // Laminar
      f = 64/Re;
    elseif Re > 4000 and Re < 10^4 then  // Transition
      f = 0.316/(Re^0.25);
    else                                 // Turbulent
      1/sqrt(f) = -2*log10(Roughness/(3.7*Dh) + 2.51/(Re*sqrt(f)));
    end if;

  end DarcyWeisbach;

  model Forrest
    "Friction factor for rectangular mini-channels according to correlation of Forrest et al."
    extends BaseClass;
    input Modelica.Units.SI.Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
    input Real phi_star "Geometrical correction" annotation(Dialog(enable = true));
    input Modelica.Units.SI.ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));

  equation
    if Re < 2000 then                    // Laminar
      f = 64/(phi_star*Re);
    else                                 // Turbulent
      // Corrected Blasius equation --> smooth tube
      f = 0.3164*(phi_star*Re)^(-1/4);
    end if;

    // Sanity check
    assert(not
              ((Re > 2e3) and (Re < 4e3)), "The Forrest correlation is not strictly valid in the transition region", AssertionLevel.warning);

  end Forrest;
  annotation (Icon(graphics={
      Rectangle(
        extent={{60,88},{80,-88}},
        lineColor={95,95,95},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid),
      Polygon(
        points={{-20,-2},{-20,20},{20,0},{-20,-20},{-20,-2}},
        lineColor={0,128,255},
        smooth=Smooth.None,
        fillColor={0,128,255},
        fillPattern=FillPattern.Solid,
        origin={23,52},
        rotation=90),
      Rectangle(
        extent={{-50,6},{50,-6}},
        lineColor={0,128,255},
        fillColor={0,128,255},
        fillPattern=FillPattern.Solid,
        origin={23,-18},
        rotation=90),
      Polygon(
        points={{-20,-2},{-20,20},{20,0},{-20,-20},{-20,-2}},
        lineColor={0,128,255},
        smooth=Smooth.None,
        fillColor={0,128,255},
        fillPattern=FillPattern.Solid,
        origin={-51,52},
        rotation=90),
      Rectangle(
        extent={{-50,6},{50,-6}},
        lineColor={0,128,255},
        fillColor={0,128,255},
        fillPattern=FillPattern.Solid,
        origin={-51,-18},
        rotation=90)}));
end DPCorrelations;
