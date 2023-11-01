within DynTherM.Components.TwoDimensional;
model RectangularChannels2D
  "Parallel rectangular channels implementing a 2D spatial discretization"

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

  Mass m_tot "Total mass";
  Mass m_fluid "Mass of fluid";
  Mass m_solid "Mass of solid walls";

  DynTherM.CustomInterfaces.DistributedFluidPort_A inlet(
    redeclare package Medium = Medium,
    N=N_channels) annotation (Placement(transformation(extent={{-18,-22},{18,22}},
                            rotation=-90,
        origin={-100,0}),                iconTransformation(extent={{-18,-36},{18,
            36}},
        rotation=-90,
        origin={-100,-7.10543e-15})));
  DynTherM.CustomInterfaces.DistributedFluidPort_B outlet(
    redeclare package Medium = Medium,
    N=N_channels) annotation (Placement(transformation(extent={{-18,-26},{18,26}},
                            rotation=-90,
        origin={100,0}),                 iconTransformation(extent={{-18,-38},{18,
            38}},
        rotation=-90,
        origin={100,0})));
  CustomInterfaces.DistributedHeatFluxPort_B solid_surface_north(
    Nx=N_cv,
    Ny=N_channels) annotation (
      Placement(transformation(extent={{-60,76},{-20,136}}),iconTransformation(
          extent={{-60,76},{-20,136}})));
  CustomInterfaces.DistributedHeatFluxPort_B solid_surface_east(
    Nx=N_cv, Ny=1) annotation (
      Placement(transformation(extent={{20,76},{60,136}}),   iconTransformation(
          extent={{20,76},{60,136}})));
  CustomInterfaces.DistributedHeatFluxPort_B solid_surface_south(
    Nx=N_cv,
    Ny=N_channels) annotation (
      Placement(transformation(extent={{-60,-136},{-20,-76}}),
                                                             iconTransformation(
          extent={{-60,-136},{-20,-76}})));
  CustomInterfaces.DistributedHeatFluxPort_B solid_surface_west(
    Nx=N_cv, Ny=1) annotation (
      Placement(transformation(extent={{20,-136},{60,-76}}), iconTransformation(
          extent={{20,-136},{60,-76}})));

equation
  m_tot = m_fluid + m_solid;
  m_fluid = sum(channel.m_fluid);
  m_solid = sum(channel.m_solid) - L*t_int*(H + 2*t_ext)*Mat.rho +
    2*L*(t_ext - t_int)*(H + 2*t_ext)*Mat.rho;

  // thermal connections
  // internal
  for i in 1:(N_channels - 1) loop
    connect(channel[i].solid_surface_east, channel[i+1].solid_surface_west);
  end for;

  // external
  connect(solid_surface_west, channel[1].solid_surface_west);
  connect(solid_surface_east, channel[N_channels].solid_surface_east);

  for i in 1:N_channels loop
    for j in 1:N_cv loop
      connect(solid_surface_north.ports[j,i], channel[i].solid_surface_north.ports[j,1]);
      connect(solid_surface_south.ports[j,i], channel[i].solid_surface_south.ports[j,1]);
    end for;
  end for;

  // boundary flow connections
  for i in 1:N_channels loop
    connect(inlet.ports[i], channel[i].inlet);
    connect(outlet.ports[i], channel[i].outlet);
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                      Rectangle(
          extent={{-80,-20},{80,-40}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
                      Rectangle(
          extent={{-80,-80},{80,-100}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
                      Rectangle(
          extent={{-80,40},{80,20}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
                      Rectangle(
          extent={{-80,100},{80,80}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Rectangle(extent={{-80,100},{80,-100}}, lineColor={0,0,0}),
        Line(
          points={{-40,80},{-40,40}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{0,80},{0,40}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{40,80},{40,40}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{-40,20},{-40,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{0,20},{0,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{40,20},{40,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{-40,-40},{-40,-80}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{0,-40},{0,-80}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{40,-40},{40,-80}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{-100,0},{-80,0}},
          color={0,0,0}),
        Line(
          points={{-100,-60},{-80,-60}},
          color={0,0,0}),
        Line(
          points={{-100,60},{-80,60}},
          color={0,0,0}),
        Line(
          points={{80,60},{100,60}},
          color={0,0,0}),
        Line(
          points={{80,0},{100,0}},
          color={0,0,0}),
        Line(
          points={{80,-60},{100,-60}},
          color={0,0,0}),
        Line(
          points={{-100,60},{-100,-60}},
          color={0,0,0}),
        Line(
          points={{100,60},{100,-60}},
          color={0,0,0})}),                    Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end RectangularChannels2D;
