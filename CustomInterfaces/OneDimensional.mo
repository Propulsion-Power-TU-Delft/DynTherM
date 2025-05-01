within DynTherM.CustomInterfaces;
package OneDimensional

  connector HeatPort1D_A "A-type 1D heat connector"
    connector HeatPort = Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a;
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    HeatPort ports[Nx];

    annotation (Icon(graphics={      Rectangle(
            extent={{-100,20},{100,-20}},
            lineColor={191,0,0},
            fillColor={191,0,0},
            fillPattern=FillPattern.Solid)}));
  end HeatPort1D_A;

  connector HeatPort1D_B "B-type 1D heat connector"
    connector HeatPort = Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b;
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    HeatPort ports[Nx];

    annotation (Icon(graphics={      Rectangle(
            extent={{-100,20},{100,-20}},
            lineColor={191,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end HeatPort1D_B;

  connector HeatFluxPort1D_A "A-type 1D heat flux connector"
    connector HeatPort = ZeroDimensional.HeatFluxPort_A;
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    HeatPort ports[Nx];
    annotation (Icon(graphics={
                           Rectangle(
            extent={{-100,20},{100,-20}},
            lineColor={255,127,0},
            fillColor={255,127,0},
            fillPattern=FillPattern.Solid)}));
  end HeatFluxPort1D_A;

  connector HeatFluxPort1D_B "B-type 1D heat flux port connector"
    connector HeatPort = ZeroDimensional.HeatFluxPort_B;
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    HeatPort ports[Nx];
    annotation (Icon(graphics={
                           Rectangle(
            extent={{-100,20},{100,-20}},
            lineColor={255,127,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end HeatFluxPort1D_B;

  connector IrradiancePort1D "1D irradiance connector"
    connector IrradiancePort = ZeroDimensional.IrradiancePort;
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    IrradiancePort ports[Nx];
    annotation (Icon(graphics={
                           Rectangle(
            extent={{-100,20},{100,-20}},
            lineColor={238,46,47},
            fillColor={255,127,0},
            fillPattern=FillPattern.Solid), Ellipse(
            extent={{-60,16},{60,-16}},
            lineColor={191,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end IrradiancePort1D;

  connector FluidPort1D_A "A-type 1D flow connector"
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    connector FluidPort = ZeroDimensional.FluidPort_A (
      redeclare package Medium=Medium);

    parameter Integer Nx(min=1) "Number of ports in x-direction";
    FluidPort ports[Nx];
    annotation (Icon(graphics={Ellipse(
            extent={{-100,-20},{100,20}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid)}));
  end FluidPort1D_A;

  connector FluidPort1D_B "B-type 1D flow connector"
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    connector FluidPort = ZeroDimensional.FluidPort_B (
      redeclare package Medium=Medium);

    parameter Integer Nx(min=1) "Number of ports in x-direction";
    FluidPort ports[Nx];
    annotation (Icon(graphics={Ellipse(
            extent={{-100,-20},{100,20}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
                               Ellipse(
            extent={{-86,-12},{86,12}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end FluidPort1D_B;
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
          textString="1D")}));
end OneDimensional;
