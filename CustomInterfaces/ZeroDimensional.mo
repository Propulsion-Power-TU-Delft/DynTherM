within DynTherM.CustomInterfaces;
package ZeroDimensional

  connector FluidPort "Flow connector"
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    flow MassFlowRate m_flow
      "Mass flow rate from the connection point into the component";
    Pressure P "Pressure in the connection point";
    stream SpecificEnthalpy h_outflow
      "Specific thermodynamic enthalpy close to the connection point if m_flow < 0";
    stream MassFraction Xi_outflow[Medium.nX]
      "Independent mixture mass fractions m_i/m close to the connection point if m_flow < 0";

    annotation (Icon(graphics), Documentation(info="<HTML>
</HTML>", revisions="<html>
<ul>
<li><i>20 Dec 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Adapted to Modelica.Media.</li>
<li><i>5 Mar 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       First release.</li>
</ul>
</html>"));
  end FluidPort;

  connector FluidPort_A "A-type flow connector"
    extends DynTherM.CustomInterfaces.ZeroDimensional.FluidPort;
    annotation (Icon(graphics={Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid)}));
  end FluidPort_A;

  connector FluidPort_B "B-type flow connector"
    extends DynTherM.CustomInterfaces.ZeroDimensional.FluidPort;
    annotation (Icon(graphics={Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid), Ellipse(
            extent={{-50,50},{50,-50}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end FluidPort_B;

  connector ExtFluidPort "Flow connector using ExternalMedia library"
    replaceable package Medium = Media.ExtMedia.CoolProp.Hydrogen constrainedby
      ExternalMedia.Media.BaseClasses.ExternalTwoPhaseMedium "Medium model" annotation(choicesAllMatching = true);

    flow MassFlowRate m_flow
      "Mass flow rate from the connection point into the component";
    Pressure P "Pressure in the connection point";
    stream SpecificEnthalpy h_outflow
      "Specific thermodynamic enthalpy close to the connection point if m_flow < 0";
    stream MassFraction Xi_outflow[Medium.nX]
      "Independent mixture mass fractions m_i/m close to the connection point if m_flow < 0";
    stream Medium.ExtraProperty C_outflow[Medium.nC]
      "Properties c_i/m close to the connection point if m_flow < 0";

    annotation (Icon(graphics), Documentation(info="<HTML>
</HTML>", revisions="<html>
<ul>
<li><i>20 Dec 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Adapted to Modelica.Media.</li>
<li><i>5 Mar 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       First release.</li>
</ul>
</html>"));
  end ExtFluidPort;

  connector ExtFluidPort_A "A-type flow connector using ExternalMedia"
    extends ZeroDimensional.ExtFluidPort;
    annotation (Icon(graphics={Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid)}));
  end ExtFluidPort_A;

  connector ExtFluidPort_B "B-type flow connector using ExternalMedia"
    extends ZeroDimensional.ExtFluidPort;
    annotation (Icon(graphics={Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid), Ellipse(
            extent={{-50,50},{50,-50}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end ExtFluidPort_B;

  connector MechanicalPort "Connector for exchange of mechanical power"
    Torque M "Torque";
    AngularVelocity omega "Angular velocity";
    annotation (Icon(graphics), Documentation(info="<HTML>
</HTML>", revisions="<html>
<ul>
<li><i>20 Dec 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Adapted to Modelica.Media.</li>
<li><i>5 Mar 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       First release.</li>
</ul>
</html>"));
  end MechanicalPort;

  connector Shaft_A "A-type mechanical connector"
    extends ZeroDimensional.MechanicalPort;
    annotation (Icon(graphics={                                                                                     Polygon(
            points={{-100,40},{-40,100},{40,100},{100,40},{100,-40},{40,
                -100},{-40,-100},{-100,-40},{-100,40}},
            lineColor={135,135,135},
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid)}));
  end Shaft_A;

  connector Shaft_B "B-type mechanical connector"
    extends ZeroDimensional.MechanicalPort;
    annotation (Icon(graphics={                                                                                     Polygon(
            points={{-100,40},{-40,100},{40,100},{100,40},{100,-40},{40,
                -100},{-40,-100},{-100,-40},{-100,40}},
            lineColor={135,135,135},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=1)}));
  end Shaft_B;

  connector HeatFluxPort "Heat flux connector"
    Temperature T "Port temperature";
    flow HeatFlux phi "Heat flux (positive if flowing from outside into the port)";
  end HeatFluxPort;

  connector HeatFluxPort_A "A-type heat flux connector"
    extends ZeroDimensional.HeatFluxPort;
    annotation (Icon(graphics={
                           Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={255,127,0},
            fillColor={255,127,0},
            fillPattern=FillPattern.Solid)}));
  end HeatFluxPort_A;

  connector HeatFluxPort_B "B-type heat flux connector"
    extends ZeroDimensional.HeatFluxPort;
    annotation (Icon(graphics={
                           Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={255,127,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end HeatFluxPort_B;

  connector IrradiancePort "Irradiance connector"
    Irradiance E_tb "Beam component of the clear-sky solar irradiance";
    Irradiance E_td "Diffuse component of the clear-sky solar irradiance";
    Irradiance E_tr "Ground reflected component of the clear-sky solar irradiance";
    Angle theta "Incidence angle";
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                    Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={191,0,0},
            fillColor={244,125,35},
            fillPattern=FillPattern.Solid), Ellipse(
            extent={{-72,72},{72,-72}},
            lineColor={191,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),                      Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end IrradiancePort;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),      Text(
          extent={{-98,98},{94,-94}},
          lineColor={0,0,0},
          textString="0D")}));
end ZeroDimensional;
