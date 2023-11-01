within DynTherM.Components.OneDimensional;
model RectangularChannels1D
  "Model emulating multiple instances of RectangularChannel1D in parallel"

  outer Components.Environment environment "Environmental properties";
  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Geometry
  parameter Modelica.Units.SI.Length L "Channel length" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length W "Width of the channel" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length H "Height of the channel" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length t_north "Thickness of north wall" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length t_east "Thickness of east wall" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length t_south "Thickness of south wall" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Length t_west "Thickness of west wall" annotation (Dialog(tab="Geometry"));

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
  parameter Integer N(min=1) "Number of longitudinal sections in which the tube is discretized";
  input Real N_channels(min=1) "Number of channels in parallel" annotation(Dialog(enable = true));

  Mass m_tot "Total mass of minichannels";
  Mass m_fluid "Mass of fluid";
  Mass m_solid "Mass of solid walls";
  Pressure dP "Pressure drop";
  HeatFlowRate Q "Heat flow rate entering the rectangular channel";

  DynTherM.CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if environment.allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
            {-94,6}},       rotation=0), iconTransformation(extent={{-110,-10},{
            -90,10}})));
  DynTherM.CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if environment.allowFlowReversal then +Modelica.Constants.inf else 0, start=
          -m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
            {106,6}},       rotation=0), iconTransformation(extent={{90,-10},{110,
            10}})));

  CustomInterfaces.DistributedHeatFluxPort_B solid_surface_north(Nx=N, Ny=1)
                                                                      annotation (
      Placement(transformation(extent={{-90,16},{-60,76}}), iconTransformation(
          extent={{-90,16},{-60,76}})));
  CustomInterfaces.DistributedHeatFluxPort_B solid_surface_east(Nx=N, Ny=1)
                                                                     annotation (
      Placement(transformation(extent={{-40,16},{-10,76}}),  iconTransformation(
          extent={{-40,16},{-10,76}})));
  CustomInterfaces.DistributedHeatFluxPort_B solid_surface_south(Nx=N, Ny=1)
                                                                      annotation (
      Placement(transformation(extent={{10,16},{40,76}}),    iconTransformation(
          extent={{10,16},{40,76}})));
  CustomInterfaces.DistributedHeatFluxPort_B solid_surface_west(Nx=N, Ny=1)
                                                                     annotation (
      Placement(transformation(extent={{60,16},{90,76}}),    iconTransformation(
          extent={{60,16},{90,76}})));
  RectangularChannel1D single_channel(
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    L=L,
    W=W,
    H=H,
    t_north=t_north,
    t_east=t_east,
    t_south=t_south,
    t_west=t_west,
    T_start_solid=T_start_solid,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    N=N) annotation (Placement(transformation(extent={{-40,-40},{40,40}})));
  Adaptors.flowScaler flowScaler_inlet(redeclare package Medium = Medium,
      scaler=N_channels)
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-70,3.55271e-15})));
  Adaptors.flowScaler flowScaler_outlet(redeclare package Medium = Medium,
      scaler=1/N_channels)
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={70,0})));
equation
  m_tot = single_channel.m_tot*N_channels;
  m_fluid = single_channel.m_fluid*N_channels;
  m_solid = single_channel.m_solid*N_channels;
  dP = single_channel.dP;
  Q = single_channel.Q*N_channels;

  connect(inlet, flowScaler_inlet.inlet) annotation (Line(points={{-100,0},{-91,
          0},{-91,5.77316e-15},{-82,5.77316e-15}}, color={0,0,0}));
  connect(flowScaler_inlet.outlet, single_channel.inlet) annotation (Line(
        points={{-58,1.33227e-15},{-49,1.33227e-15},{-49,0},{-40,0}}, color={0,0,
          0}));
  connect(single_channel.outlet, flowScaler_outlet.inlet) annotation (Line(
        points={{40,0},{49,0},{49,2.22045e-15},{58,2.22045e-15}}, color={0,0,0}));
  connect(flowScaler_outlet.outlet, outlet) annotation (Line(points={{82,-2.22045e-15},
          {91,-2.22045e-15},{91,0},{100,0}}, color={0,0,0}));
  connect(single_channel.solid_surface_north, solid_surface_north) annotation (
      Line(points={{-30,18.4},{-30,30},{-75,30},{-75,46}}, color={255,127,0}));
  connect(single_channel.solid_surface_east, solid_surface_east) annotation (
      Line(points={{-10,18.4},{-10,30},{-25,30},{-25,46}}, color={255,127,0}));
  connect(single_channel.solid_surface_south, solid_surface_south) annotation (
      Line(points={{10,18.4},{10,30},{25,30},{25,46}}, color={255,127,0}));
  connect(single_channel.solid_surface_west, solid_surface_west) annotation (
      Line(points={{30,18.4},{30,30},{75,30},{75,46}}, color={255,127,0}));
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
          preserveAspectRatio=false)),
    Documentation(info="<html>
</html>"));
end RectangularChannels1D;
