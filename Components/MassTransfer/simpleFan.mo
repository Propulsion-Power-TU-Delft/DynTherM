within ThermalManagement.Components.MassTransfer;
model simpleFan
  package Medium = Modelica.Media.Air.MoistAir;
  outer ThermalManagement.Components.Environment environment "Environmental properties";
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
     "= true to allow flow reversal, false restricts to design direction";
  parameter Real eta_is "Isentropic efficiency at design point";
  parameter Real eta_m "Mechanical efficiency";
  parameter Modelica.Units.SI.AngularVelocity omega_nom
    "Nominal rotational speed" annotation (Dialog(tab="Nominal values"));
  parameter Modelica.Units.SI.VolumeFlowRate volFlow_nom
    "Nominal volumetric flow rate" annotation (Dialog(tab="Nominal values"));
  parameter Modelica.Units.SI.SpecificEnergy Head_nom
    "Nominal head provided by the fan"
    annotation (Dialog(tab="Nominal values"));
  parameter Modelica.Units.SI.SpecificEnthalpy h_start=1e5
    "Specific enthalpy - start value"
    annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Density rho_start=1.1 "Density - start value"
    annotation (Dialog(tab="Initialization"));

  Medium.ThermodynamicState state_in "Inlet thermodynamic state";
  Modelica.Units.SI.Density rho(start=rho_start)
    "Density - constant, since the fluid is incompressible";
  Modelica.Units.SI.Power W "Power Consumption without mechanical losses";
  Modelica.Units.SI.Power Pm "Mechanical power Consumption";
  Modelica.Units.SI.MassFlowRate massFlow "Mass flow rate";
  Modelica.Units.SI.VolumeFlowRate volFlow "Volumetric flow rate";
  Real beta "Compression ratio";

  replaceable
    ThermalManagement.Components.MassTransfer.FanCharacteristics.FlowCharacteristics.dummyFlow
    flowModel constrainedby
    ThermalManagement.Components.MassTransfer.FanCharacteristics.BaseClass(
    omega_nom=omega_nom,
    omega=shaft.omega,
    volFlow_nom=volFlow_nom,
    volFlow=volFlow,
    Head_nom=Head_nom) annotation (choicesAllMatching=true);

   CustomInterfaces.Shaft_A shaft annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
   CustomInterfaces.FluidPort_A inlet(
     h_outflow(start=h_start),
     m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0)) annotation (Placement(transformation(
           extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{
             -90,10}})));
   CustomInterfaces.FluidPort_B outlet(
     h_outflow(start=h_start),
     m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0)) annotation (Placement(transformation(
           extent={{90,-10},{110,10}}), iconTransformation(extent={{90,-10},{110,
             10}})));
equation
   state_in = Medium.setState_phX(inlet.P, inStream(inlet.h_outflow), inStream(inlet.Xi_outflow));
   rho = Medium.density(state_in);

   // Boundary conditions
   massFlow = inlet.m_flow "Fan total flow rate";
   volFlow = massFlow/rho;

   // Mass balance
   inlet.m_flow + outlet.m_flow = 0;

   // Independent composition mass balances
   inlet.Xi_outflow = inStream(outlet.Xi_outflow);
   outlet.Xi_outflow = inStream(inlet.Xi_outflow);

   // Energy balance
   outlet.h_outflow = inStream(inlet.h_outflow) + W/massFlow "Energy balance for w > 0";
   inlet.h_outflow = inStream(outlet.h_outflow) + W/massFlow "Energy balance for w < 0";

   // Flow characteristics
   beta = outlet.P/inlet.P;
   flowModel.Head = (outlet.P - inlet.P)/rho;
   W = (outlet.P - inlet.P)*volFlow/eta_is;
   Pm = W/eta_m;
   shaft.M*shaft.omega + Pm = 0;

 annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
     Ellipse(extent={{-60,60},{60,-60}}, lineColor={0,0,0},
         lineThickness=0.5),
     Line(points={{-30,52},{44,40}}, color={0,0,0},
         thickness=0.5),
     Line(points={{-30,-52},{44,-40}}, color={0,0,0},
         thickness=0.5),
     Polygon(
       points={{4,0},{0,0},{-20,26},{-14,24},{4,0}},
       lineColor={0,0,0},
       smooth=Smooth.Bezier),
     Polygon(
       points={{-20,-24},{-16,-24},{4,2},{-2,0},{-20,-24}},
       lineColor={0,0,0},
       smooth=Smooth.Bezier),
     Ellipse(extent={{0,2},{30,-2}}, lineColor={0,0,0}),
       Line(points={{-100,0},{-60,0}}, color={0,0,0}),
       Line(points={{60,0},{92,0}}, color={0,0,0}),
       Rectangle(extent={{-4,90},{4,60}}, lineColor={0,0,0})}), Diagram(
       coordinateSystem(preserveAspectRatio=false)));
end simpleFan;
