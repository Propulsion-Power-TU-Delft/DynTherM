within DynTherM.Components.OneDimensional;
model ParallelCircularColdPlate
  "Cold plate heat exchanger featuring parallel circular mini-channels"

  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material used for the plate" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  model Channel =
      DynTherM.Components.OneDimensional.CircularAsymmetricChannel1D;

  // Geometry
  parameter Length L "Length" annotation (Dialog(tab="Geometry"));
  parameter Length W_plate "Width of the plate" annotation (Dialog(tab="Geometry"));
  parameter Length H_plate=2e-3 "Height of the plate" annotation (Dialog(tab="Geometry"));
  parameter Length R_int=0.3e-3 "Internal radius of mini-channels" annotation (Dialog(tab="Geometry"));
  parameter Length Roughness=0.015*10^(-3) "Channel roughness" annotation (Dialog(tab="Geometry"));

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
  parameter Integer N_channels(min=1) "Number of channels in parallel";

  Channel channel[N_channels](
    redeclare model Mat = Mat,
    redeclare package Medium = Medium,
    each N_cv=N_cv,
    each N_channels=1,
    each L=L,
    each R_ext_north=R_ext_v,
    each R_ext_east=R_ext_h,
    each R_ext_south=R_ext_v,
    each R_ext_west=R_ext_h,
    each R_int=R_int,
    each Roughness=Roughness,
    each T_start_solid=T_start_plate,
    each T_start_fluid=T_start_fluid,
    each P_start=P_start,
    each X_start=X_start,
    each state_start=state_start,
    each m_flow_start=m_flow_start,
    each u_start=u_start,
    each rho_start=rho_start,
    each dP_start=dP_start,
    each Re_start=Re_start,
    each Pr_start=Pr_start,
    each initOpt=initOpt,
    each allowFlowReversal=allowFlowReversal);

  Length d "Distance between adjacent mini-channels";
  Length R_ext_v "External radius of mini-channels - vertical direction";
  Length R_ext_h "External radius of mini-channels - horizontal direction";

  CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0,
      start=m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start))
    annotation (Placement(transformation(extent={{-120,
            -20},{-80,20}}, rotation=0), iconTransformation(extent={{-110,-10},
            {-90,10}})));
  CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0,
      start=-m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start))
    annotation (Placement(transformation(extent={{80,-20},
            {120,20}},      rotation=0), iconTransformation(extent={{90,-10},{
            110,10}})));
  CustomInterfaces.DistributedHeatPort_A upper_surface(Nx=N_cv, Ny=N_channels)
    annotation (
      Placement(transformation(extent={{-16,44},{16,76}}),  iconTransformation(
          extent={{-30,38},{30,98}})));
  CustomInterfaces.DistributedHeatPort_A lower_surface(Nx=N_cv, Ny=N_channels)
    annotation (Placement(transformation(extent={{-16,-76},{16,-44}}),
        iconTransformation(extent={{-30,-98},{30,-38}})));

equation
  R_ext_h = R_int + d/2;
  W_plate = N_channels*2*R_ext_h;
  H_plate = 2*R_ext_v;

  // boundary thermal connections
  for j in 1:N_channels loop
    for i in 1:N_cv loop
      connect(upper_surface.ports[i,j], channel[j].cv[i].solid_surface_north);
      connect(lower_surface.ports[i,j], channel[j].cv[i].solid_surface_south);
    end for;
  end for;

  // internal thermal connections
  for j in 1:(N_channels-1) loop
    for i in 1:N_cv loop
      connect(channel[j].cv[i].solid_surface_east, channel[j+1].cv[i].solid_surface_west);
    end for;
  end for;

  // boundary flow connections
  for i in 1:N_channels loop
    connect(inlet, channel[i].inlet);
    connect(channel[i].outlet, outlet);
  end for;

  // Sanity check
  assert(R_ext_v > R_int, "Vertical external radius must be greater than internal radius");
  assert(R_ext_h > R_int, "Horizontal external radius must be greater than internal radius");
  assert(W_plate > N_channels*2*R_int, "Number of mini-channels is too large for the prescribed plate width and channel internal radius");

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
<p>The circular mini-channels are discretized along the longitudinal direction.</p>
<p><img src=\"modelica://DynTherM/Figures/Circular_cold_plate.png\"/></p>
</html>"));
end ParallelCircularColdPlate;
