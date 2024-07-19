within DynTherM.Systems.Battery;
model ColdPlateRectangularSeries
  "Cold plate heat exchanger featuring rectangular mini-channels in series"

  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  model Channel = Components.OneDimensional.RectangularChannel1D "Cooling channel";

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Boolean use_inertia=true
    "= true to place small plenum in between two adjacent CVs to improve numerical stability,
    at the expense of higher computational cost";

  // Geometry
  parameter Length L "Length of the channels" annotation (Dialog(tab="Geometry"));
  input Length W "Width of the channels - internal section" annotation (Dialog(tab="Geometry", enable=true));
  input Length H "Height of the channels - internal section" annotation (Dialog(tab="Geometry", enable=true));
  input Length t_vertical "Thickness of the channels - vertical direction" annotation (Dialog(tab="Geometry", enable=true));
  input Length t_horizontal "Thickness of the channels - horizontal direction" annotation (Dialog(tab="Geometry", enable=true));
  parameter Volume V_inertia=1e-10 "Volume of the plenum placed between two consecutive control volumes" annotation (Dialog(tab="Geometry"));

  // Initialization
  parameter Temperature T_start_solid=298.15
    "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start_fluid=298.15
    "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=3e5
    "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start=
    Medium.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter MassFlowRate m_flow_start=0.05 "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start=3 "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
  parameter Density rho_start=1e3 "Density - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure dP_start=1e4 "Pressure drop - start value" annotation (Dialog(tab="Initialization"));
  parameter ReynoldsNumber Re_start=5e3 "Reynolds number - start value" annotation (Dialog(tab="Initialization"));
  parameter PrandtlNumber Pr_start=7 "Prandtl number - start value" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N_channels(min=3)=3 "Number of channels in series";
  parameter Integer N_cv=1 "Number of longitudinal sections in which the channels are discretized";

  Channel channel[N_channels](
    redeclare model Mat=Mat,
    redeclare package Medium=Medium,
    each L=L,
    each W=W,
    each H=H,
    each t_north=t_vertical,
    each t_south=t_vertical,
    each t_east=t_horizontal,
    each t_west=t_horizontal,
    each T_start_solid=T_start_solid,
    each T_start_fluid=T_start_fluid,
    each P_start=P_start,
    each X_start=X_start,
    each state_start=state_start,
    each m_flow_start=m_flow_start,
    each m_flow_mc_start=m_flow_start,
    each u_start=u_start,
    each rho_start=rho_start,
    each dP_start=dP_start,
    each Re_start=Re_start,
    each Pr_start=Pr_start,
    each N_channels=1,
    each N_cv=N_cv,
    each V_inertia=V_inertia,
    each use_inertia=use_inertia,
    each allowFlowReversal=allowFlowReversal,
    each initOpt=initOpt);

  CustomInterfaces.DistributedHeatPort_A upper_surface(
    Nx=N_channels,
    Ny=N_cv)
    annotation (Placement(transformation(
        extent={{-20,-14},{20,14}},
        rotation=-90,
        origin={-100,0}), iconTransformation(extent={{-30,-98},{30,-38}})));
  CustomInterfaces.DistributedHeatPort_A lower_surface(
    Nx=N_channels,
    Ny=N_cv)
    annotation (
      Placement(transformation(
        extent={{-20,-14},{20,14}},
        rotation=90,
        origin={100,0}), iconTransformation(extent={{-30,38},{30,98}})));
  CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(
        transformation(extent={{-106,94},{-94,106}}), iconTransformation(extent
          ={{-110,42},{-90,62}})));
  CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=
          -m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(
        transformation(extent={{-106,54},{-94,66}}), iconTransformation(extent={
            {-110,18},{-90,38}})));

  Mass m_tot "Total mass";
  Mass m_fluid "Mass of fluid";
  Mass m_solid "Mass of solid walls";
  Pressure dP "Pressure drop";
  HeatFlowRate Q "Heat flow rate - positive entering";

protected
  final parameter Integer iter = integer(N_channels/2 - 1);

equation
  m_tot = sum(channel.m_tot);
  m_fluid = sum(channel.m_fluid);
  m_solid = sum(channel.m_solid);
  dP = sum(channel.dP);
  Q = sum(channel.Q);

  // Flow connections
  // Internal
  for i in 1:(N_channels-1) loop
    connect(channel[i].outlet, channel[i+1].inlet);
  end for;

  // External
  connect(inlet, channel[1].inlet);
  connect(outlet, channel[end].outlet);

  // Thermal connections
  // Internal
  connect(channel[1].solid_surface_east, channel[end].solid_surface_east);

  for i in 1:iter loop
    connect(channel[i*2].solid_surface_east, channel[i*2 + 1].solid_surface_east);
    connect(channel[i*2 + 1].solid_surface_west, channel[i*2 + 2].solid_surface_west);
  end for;

  // External
  for i in 1:N_channels loop
    for j in 1:N_cv loop
      connect(channel[i].solid_surface_north.ports[j, 1], upper_surface.ports[i, j]);
      connect(channel[i].solid_surface_south.ports[j, 1], lower_surface.ports[i, j]);
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},
            {100,120}}), graphics={
        Rectangle(extent={{-100,60},{100,-60}}, lineColor={0,0,0}),
        Rectangle(
          extent={{-100,54},{94,46}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-100,34},{76,26}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-94,-46},{94,-54}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-54,4},{54,-4}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder,
          origin={90,0},
          rotation=-90),
        Rectangle(
          extent={{-94,14},{76,6}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-94,-6},{76,-14}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-94,-26},{76,-34}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-14,4},{14,-4}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder,
          origin={-90,-40},
          rotation=-90),
        Rectangle(
          extent={{-14,4},{14,-4}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder,
          origin={76,-20},
          rotation=-90),
        Rectangle(
          extent={{-14,4},{14,-4}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder,
          origin={-90,0},
          rotation=-90),
        Rectangle(
          extent={{-14,4},{14,-4}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder,
          rotation=-90,
          origin={76,20})}),                                     Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{100,120}})),
    experiment(
      StopTime=1200,
      Interval=1,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>The number of channels can be freely selected by the user, while the disposition is fixed, see image below.</p>
<p><img src=\"modelica://TMS/Figures/cold_plate.png\"/></p>
</html>"));
end ColdPlateRectangularSeries;
