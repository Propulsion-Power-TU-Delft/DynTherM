within DynTherM.Components.HeatTransfer;
model SphereConduction "Dynamic model of conduction in a hollow sphere"

  replaceable model Mat=DynTherM.Materials.Aluminium constrainedby
    DynTherM.Materials.Properties "Material choice" annotation (choicesAllMatching=true);

    parameter Real coeff=1 "Fraction of sphere with active heat transfer";
    parameter Length R_ext "External radius";
    parameter Length R_int "Internal radius";
    parameter Real AR "Aspect ratio";
    parameter Temperature Tstart=288.15 "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter DynTherM.Choices.InitOpt initOpt "Initialization option" annotation (Dialog(tab="Initialization"));

    final parameter Real coeff_A=coeff*AR*(R_int/R_eq_int)
      "Fraction of equivalent sphere area with active heat transfer";
    final parameter Length R_eq_ext=R_ext*(1+AR^2)/(2*AR)
      "Radius of equivalent sphere when AR < 1";
    final parameter Length R_eq_int=R_int*(1+AR^2)/(2*AR)
      "Radius of equivalent sphere when AR < 1";
    final parameter Mass m=2*coeff*Mat.rho*(pi*(AR*R_ext)^2*(R_eq_ext - 1/3*(AR*R_ext)) -
      pi*(AR*R_int)^2*(R_eq_int - 1/3*(AR*R_int))) "Mass";
    final parameter Modelica.Units.SI.HeatCapacity Cm=m*Mat.cm "Heat capacity";
    constant Real pi=Modelica.Constants.pi;

    Temperature T_vol "Average temperature of the sphere";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet annotation (Placement(transformation(extent={{-14,20},{14,48}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet annotation (Placement(transformation(extent={{-14,-48},{14,-20}})));

equation
  assert(R_eq_ext > R_eq_int, "External radius must be greater than internal radius");

    Cm*der(T_vol) = inlet.Q_flow + outlet.Q_flow "Energy balance";
    inlet.Q_flow = coeff_A*(4*pi*Mat.lambda*R_eq_ext*R_eq_int)*(inlet.T - T_vol)/
      ((R_eq_ext + R_eq_int)/2 - R_eq_int) "Heat conduction through the internal half-thickness";
    outlet.Q_flow = coeff_A*(4*pi*Mat.lambda*R_eq_ext*R_eq_int)*(outlet.T - T_vol)/
      (R_eq_ext - (R_eq_ext + R_eq_int)/2) "Heat conduction through the external half-thickness";

initial equation
    if initOpt == DynTherM.Choices.InitOpt.steadyState then
      der(T_vol) = 0;
    elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
      T_vol = Tstart;
    else
      // do nothing
    end if;
  annotation (
    Icon(graphics={   Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Text(
          extent={{-44,36},{46,-36}},
          lineColor={255,255,255},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="SPHERE")}),
    Documentation(info="<html>
<p>If the aspect ratio is smaller than 1, the model represents a dome, i.e., a cutoff of a half-sphere.</p>
<p>With reference to the figure, the aspect ratio is defined as h/R_tank_int.</p>
<p><img src=\"modelica://DynTherM/Figures/DomeAspectRatio.PNG\"/></p>
<p><br><b>Reference:</b></p>
<p>[1] M. Swart. &quot;Exploration of a Reverse Turbo-Brayton Cryocooler for Carbon Neutral Aeronautical Applications&quot;, MSc Thesis, TU Delft, 2024. </p>
</html>"));
end SphereConduction;
