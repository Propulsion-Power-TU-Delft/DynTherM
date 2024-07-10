within DynTherM.Components.Battery;
model ParallelRectangularColdPlate
  "Cold plate heat exchanger featuring parallel rectangular mini-channels"

  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material used for the plate" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Geometry
  parameter Length L "Length" annotation (Dialog(tab="Geometry"));
  parameter Length W_plate "Width of the plate" annotation (Dialog(tab="Geometry"));
  parameter Length W_channel=12e-3 "Width of one mini-channel" annotation (Dialog(tab="Geometry"));
  parameter Length H_plate=2e-3 "Height of the plate" annotation (Dialog(tab="Geometry"));
  parameter Length H_channel=0.6e-3 "Height of one mini-channel" annotation (Dialog(tab="Geometry"));
  parameter Length d=14e-3 "Distance between adjacent mini-channels" annotation (Dialog(tab="Geometry"));
  parameter Volume V_inertia=1e-10 "Volume of the plenum placed between two consecutive control volumes" annotation (Dialog(tab="Geometry"));

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  // Initialization
  parameter Temperature T_start_plate=288.15 "Temperature of the plate - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start_fluid=288.15 "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325 "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start=
    Medium.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter MassFlowRate m_flow_start=1 "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start=2 "Flow velocity within one channel - start value" annotation (Dialog(tab="Initialization"));
  parameter Density rho_start=1 "Density - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure dP_start=0.1e5 "Pressure drop - start value" annotation (Dialog(tab="Initialization"));
  parameter ReynoldsNumber Re_start=20e3 "Reynolds number - start value" annotation (Dialog(tab="Initialization"));
  parameter PrandtlNumber Pr_start=1.5 "Prandtl number - start value" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N_cv(min=1) "Number of longitudinal control volumes";

  Length t_plate "Plate thickness";

  CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0,
      start=m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-120,
            -20},{-80,20}}, rotation=0), iconTransformation(extent={{-110,-10},
            {-90,10}})));
  CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0,
      start=-m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{80,-20},
            {120,20}},      rotation=0), iconTransformation(extent={{90,-10},{
            110,10}})));
  CustomInterfaces.DistributedHeatPort_A upper_surface(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(extent={{-46,34},{-14,66}}), iconTransformation(
          extent={{-30,38},{30,98}})));
  OneDimensional.RectangularChannel1D channels(
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    allowFlowReversal=allowFlowReversal,
    L=L,
    W=W_channel,
    H=H_channel,
    t_north=t_plate,
    t_east=d/2,
    t_south=t_plate,
    t_west=d/2,
    V_inertia=V_inertia,
    T_start_solid=T_start_plate,
    T_start_fluid=T_start_fluid,
    P_start=P_start,
    X_start=X_start,
    state_start=state_start,
    m_flow_start=m_flow_start,
    u_start=u_start,
    rho_start=rho_start,
    dP_start=dP_start,
    initOpt=initOpt,
    Re_start=Re_start,
    Pr_start=Pr_start,
    N_cv=N_cv,
    N_channels=ceil(N_channels_real))
    annotation (Placement(transformation(extent={{-40,-40},{40,40}})));

  CustomInterfaces.DistributedHeatPort_A lower_surface(Nx=N_cv, Ny=1)
    annotation (Placement(transformation(extent={{-6,34},{26,66}}),
        iconTransformation(extent={{-30,-98},{30,-38}})));

protected
  Real N_channels_real;

equation
  W_plate = N_channels_real*(W_channel + d);
  H_plate = H_channel + 2*t_plate;

  connect(inlet, channels.inlet)
    annotation (Line(points={{-100,0},{-40,0}}, color={0,0,0}));
  connect(channels.outlet, outlet)
    annotation (Line(points={{40,0},{100,0}}, color={0,0,0}));
  connect(channels.solid_surface_north, upper_surface)
    annotation (Line(points={{-30,18.4},{-30,50}}, color={191,0,0}));
  connect(channels.solid_surface_south, lower_surface)
    annotation (Line(points={{10,18.4},{10,50}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-100,60},{100,-60}}, lineColor={0,0,0}),
        Rectangle(
          extent={{-94,54},{-80,-54}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.VerticalCylinder),
        Rectangle(
          extent={{80,54},{94,-54}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.VerticalCylinder),
        Rectangle(
          extent={{-80,2},{80,-2}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-80,12},{80,8}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-80,22},{80,18}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-80,42},{80,38}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-80,32},{80,28}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-80,52},{80,48}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-80,-8},{80,-12}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-80,-18},{80,-22}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-80,-28},{80,-32}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-80,-38},{80,-42}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-80,-48},{80,-52}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder)}), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The rectangular mini-channels are discretized along the longitudinal direction.</p>
<p>The heat transfer between adjacent mini-channels is disregarded to reduce the computational cost.</p>
<p>Therefore, the temperature at each longitudinal control volume is uniform along the transversal direction.</p>
<p><b>Reference</b>: </p>
<p>[1] Y. Chen, et al. &quot;Bidirectional symmetrical parallel mini-channel cold plate for energy efficient cooling of large battery packs&quot;, Energy, 2021.</p>
<p><img src=\"modelica://DynTherM/Figures/cold_plate.png\"/></p>
</html>"));
end ParallelRectangularColdPlate;
