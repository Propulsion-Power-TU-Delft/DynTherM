within DynTherM.Components;
package TwoDimensional
  "Package collecting the components modeling coupled heat and mass transfer and featuring a two dimensional spatial discretization"

  model RectangularChannels2D
    "Parallel rectangular channels implementing two-dimensional spatial discretization"

    outer Components.Environment environment "Environmental properties";
    replaceable model Mat = Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
    model Channel = OneDimensional.RectangularChannel1D "Model of one channel";

    // Geometry
    parameter Modelica.Units.SI.Length L "Channel length" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length W "Width of one channel" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length H "Height of one channel" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length t_ext "Thickness of external walls" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length t_int "Thickness of internal walls" annotation (Dialog(tab="Geometry"));

    // Initialization
    parameter Modelica.Units.SI.Temperature T_start_solid=288.15
      "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Temperature T_start_fluid=288.15
      "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Pressure P_start=101325
      "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.MassFraction X_start[Medium.nX]=Medium.reference_X
      "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
    parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start_fluid, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
      "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=environment.initOpt
      "Initialization option" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer N_cv(min=1) "Number of longitudinal sections in which each channel is discretized";
    parameter Integer N_channels(min=1) "Number of channels";

    Channel channel[N_channels](
      redeclare model Mat = Mat,
      redeclare package Medium = Medium,
      each N=N_cv,
      each L=L,
      each W=W,
      each H=H,
      each t_north=t_ext,
      each t_south=t_ext,
      each t_east=t_int,
      each t_west=t_int,
      each T_start_solid=T_start_solid,
      each T_start_fluid=T_start_fluid,
      each P_start=P_start,
      each X_start=X_start,
      each state_start=state_start,
      each m_flow_start=m_flow_start,
      each initOpt=initOpt);

    DynTherM.CustomInterfaces.DistributedFluidPort_A inlet(
      redeclare package Medium = Medium,
      N=N_channels) annotation (Placement(transformation(extent={{-106,-6},
              {-94,6}},       rotation=0), iconTransformation(extent={{-18,-36},{18,
              36}},
          rotation=-90,
          origin={-100,-7.10543e-15})));
    DynTherM.CustomInterfaces.DistributedFluidPort_B outlet(
      redeclare package Medium = Medium,
      N=N_channels) annotation (Placement(transformation(extent={{94,-6},
              {106,6}},       rotation=0), iconTransformation(extent={{-18,-38},{18,
              38}},
          rotation=-90,
          origin={100,0})));
    CustomInterfaces.DistributedHeatFluxPort_B solid_surface_north(N=N_cv) annotation (
        Placement(transformation(extent={{-90,16},{-60,76}}), iconTransformation(
            extent={{-90,16},{-60,76}})));
    CustomInterfaces.DistributedHeatFluxPort_B solid_surface_east(N=N_cv) annotation (
        Placement(transformation(extent={{-40,16},{-10,76}}),  iconTransformation(
            extent={{-40,16},{-10,76}})));
    CustomInterfaces.DistributedHeatFluxPort_B solid_surface_south(N=N_cv) annotation (
        Placement(transformation(extent={{10,16},{40,76}}),    iconTransformation(
            extent={{10,16},{40,76}})));
    CustomInterfaces.DistributedHeatFluxPort_B solid_surface_west(N=N_cv) annotation (
        Placement(transformation(extent={{60,16},{90,76}}),    iconTransformation(
            extent={{60,16},{90,76}})));

  equation
    // thermal connections
    // internal
    for i in 1:(N_channels - 1) loop
      connect(channel[i].solid_surface_east, channel[i+1].solid_surface_west);
    end for;

    // external
    connect(solid_surface_west, channel[1].solid_surface_west);
    connect(solid_surface_east, channel[N_channels].solid_surface_east);

    for i in 1:N_channels loop
      connect(solid_surface_north, channel[i].solid_surface_north);
      connect(solid_surface_south, channel[i].solid_surface_south);
    end for;

    // boundary flow connections
    for i in 1:N_channels loop
      connect(inlet.ports[i], channel[i].inlet);
      connect(outlet.ports[i], channel[i].outlet);
    end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                        Rectangle(
            extent={{-100,40},{100,20}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
                        Rectangle(
            extent={{-100,-20},{100,-40}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Rectangle(extent={{-100,20},{100,-20}}, lineColor={0,0,0}),
          Line(
            points={{-60,20},{-60,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-20,20},{-20,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{20,20},{20,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{60,20},{60,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash)}),         Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end RectangularChannels2D;
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
          extent={{-94,98},{98,-94}},
          lineColor={0,0,0},
          textString="2D")}));
end TwoDimensional;
