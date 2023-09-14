within DynTherM.Components;
package TwoDimensional

  model TubePortion "1D thermal model of 1/4 portion of a tube"

    replaceable model Mat=DynTherM.Materials.Aluminium constrainedby
      DynTherM.Materials.Properties "Material choice" annotation (choicesAllMatching=true);

    parameter Modelica.Units.SI.Length L "Tube length";
    parameter Modelica.Units.SI.Length R_ext "Tube external radius";
    parameter Modelica.Units.SI.Length R_int "Tube internal radius";
    constant Real pi=Modelica.Constants.pi;

    // Initialization
    parameter Modelica.Units.SI.Temperature T_start "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter DynTherM.Choices.InitOpt initOpt "Initialization option" annotation (Dialog(tab="Initialization"));

    final parameter Modelica.Units.SI.Mass
      m=Mat.rho*L*pi*(R_ext^2 - R_int^2)/4 "Mass of the tube portion";
    final parameter Modelica.Units.SI.HeatCapacity Cm=m*Mat.cm
      "Heat capacity of the tube portion";
    Modelica.Units.SI.Temperature T_vol(start=T_start)
      "Average temperature of the tube portion";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a int annotation (Placement(
          transformation(extent={{-10,10},{10,30}}),  iconTransformation(extent={{-10,10},
              {10,30}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b ext annotation (Placement(
          transformation(extent={{30,30},{50,50}}), iconTransformation(extent={{-10,70},
              {10,90}})));

  equation

    // Heat transfer
    Cm*der(T_vol) = int.Q_flow + ext.Q_flow "Energy balance";
    int.Q_flow = (Mat.lambda*2*pi*L/4*(int.T - T_vol))/
      Modelica.Math.log((R_int + R_ext)/(2*R_int))
      "Heat conduction through the internal half-thickness";
    ext.Q_flow = (Mat.lambda*2*pi*L/4*(ext.T - T_vol))/
      Modelica.Math.log((2*R_ext)/(R_int + R_ext))
      "Heat conduction through the external half-thickness";

  initial equation
    if initOpt == DynTherM.Choices.InitOpt.steadyState then
      der(T_vol) = 0;
    elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
      T_vol = T_start;
    else
      // do nothing
    end if;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{-40,-60},{40,20}},
            lineColor={0,0,0},
            startAngle=45,
            endAngle=135,
            closure=EllipseClosure.None,
            lineThickness=0.5),
          Ellipse(
            extent={{-100,-120},{100,80}},
            lineColor={0,0,0},
            startAngle=44,
            endAngle=136,
            closure=EllipseClosure.None,
            lineThickness=0.5),
          Line(
            points={{-72,50},{-28,8}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{28,8},{72,50}},
            color={0,0,0},
            thickness=0.5)}),                                      Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/Figures/CylindricalCell.png\"/></p>
<p>Assumptions:</p>
<p>1. Heat transfer is considered only in the radial direction</p>
<p><br>References:</p>
<p>[1] M. Macdonald. Early Design Stage Evaluation of Thermal Performance of Battery Heat Acuisition System of a Hybrid Electric Aircraft, 2020.</p>
</html>"));
  end TubePortion;

  model TubeSection

    replaceable model Mat=DynTherM.Materials.Aluminium constrainedby
      DynTherM.Materials.Properties "Material choice" annotation (choicesAllMatching=true);

    parameter Modelica.Units.SI.Length L "Tube length";
    parameter Modelica.Units.SI.Length R_ext "Tube external radius";
    parameter Modelica.Units.SI.Length R_int "Tube internal radius";

    // Initialization
    parameter Modelica.Units.SI.Temperature T_start
      "Temperature - start value" annotation (Dialog(tab="Initialization"));
    parameter DynTherM.Choices.InitOpt initOpt
      "Initialization option" annotation (Dialog(tab="Initialization"));

    TwoDimensional.TubePortion northPortion(
      redeclare model Mat = Mat,
      L = L,
      R_ext = R_ext,
      R_int = R_int,
      T_start=T_start,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{-60,-28},{60,92}})));
    TwoDimensional.TubePortion southPortion(
      redeclare model Mat = Mat,
      L = L,
      R_ext = R_ext,
      R_int = R_int,
      T_start=T_start,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{-60,28},{60,-92}})));
    TwoDimensional.TubePortion eastPortion(
      redeclare model Mat = Mat,
      L = L,
      R_ext = R_ext,
      R_int = R_int,
      T_start=T_start,
      initOpt=initOpt) annotation (Placement(transformation(
          extent={{-60,-60},{60,60}},
          rotation=-90,
          origin={32,0})));
    TwoDimensional.TubePortion westPortion(
      redeclare model Mat = Mat,
      L = L,
      R_ext = R_ext,
      R_int = R_int,
      T_start=T_start,
      initOpt=initOpt) annotation (Placement(transformation(
          extent={{-60,-60},{60,60}},
          rotation=90,
          origin={-32,0})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b E annotation (Placement(
          transformation(extent={{88,-12},{112,12}}), iconTransformation(extent={{90,-10},
              {110,10}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b N annotation (Placement(
          transformation(extent={{-14,86},{10,110}}), iconTransformation(extent={{-10,90},
              {10,110}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b W annotation (Placement(
          transformation(extent={{-114,-14},{-90,10}}), iconTransformation(extent={{-110,
              -10},{-90,10}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b S annotation (Placement(
          transformation(extent={{-12,-88},{12,-112}}), iconTransformation(extent={{-10,
              -110},{10,-90}})));

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a int annotation (Placement(
          transformation(extent={{-12,-12},{12,12}}), iconTransformation(extent={{-10,-10},
              {10,10}})));
  equation
    connect(eastPortion.ext, E) annotation (Line(points={{80,-8.88178e-15},{88,-8.88178e-15},
            {88,0},{100,0}}, color={191,0,0}));
    connect(S, southPortion.ext)
      annotation (Line(points={{0,-100},{0,-80}}, color={191,0,0}));
    connect(W, westPortion.ext) annotation (Line(points={{-102,-2},{-92,-2},{-92,3.10862e-15},
            {-80,3.10862e-15}}, color={191,0,0}));
    connect(N, northPortion.ext)
      annotation (Line(points={{-2,98},{-2,90},{0,90},{0,80}},
                                                color={191,0,0}));
    connect(westPortion.int, int) annotation (Line(points={{-44,4.44089e-16},{
            -22,4.44089e-16},{-22,0},{0,0}}, color={191,0,0}));
    connect(northPortion.int, int)
      annotation (Line(points={{0,44},{0,0}}, color={191,0,0}));
    connect(int, eastPortion.int) annotation (Line(points={{0,0},{22,0},{22,
            -2.22045e-15},{44,-2.22045e-15}}, color={191,0,0}));
    connect(southPortion.int, int)
      annotation (Line(points={{0,-44},{0,0}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
                                            Ellipse(
            extent={{-64,64},{66,-66}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),                      Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end TubeSection;
end TwoDimensional;
