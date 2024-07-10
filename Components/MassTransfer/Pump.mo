within DynTherM.Components.MassTransfer;
model Pump "Model of a pump"
  replaceable package Medium = Media.IncompressibleTableBased.MEG(X=0.5) "Medium model" annotation(choicesAllMatching = true);

  parameter Boolean allowFlowReversal=true
     "= true to allow flow reversal, false restricts to design direction";

  parameter Real eta_m=0.99 "Mechanical efficiency";
  parameter AngularVelocity omega_nom "Nominal rotational speed" annotation (Dialog(tab="Nominal values"));
  parameter VolumeFlowRate volFlow_nom "Nominal volumetric flow rate" annotation (Dialog(tab="Nominal values"));
  parameter SpecificEnergy Head_nom "Nominal head provided by the fan" annotation (Dialog(tab="Nominal values"));
  parameter Real eta_is=0.6 "Nominal isentropic efficiency" annotation (Dialog(tab="Nominal values"));

  // Initialization
  parameter SpecificEnthalpy h_start=1e5 "Specific enthalpy - start value" annotation (Dialog(tab="Initialization"));
  parameter Density rho_start=1.1 "Density - start value" annotation (Dialog(tab="Initialization"));

  Medium.ThermodynamicState state_in "Inlet thermodynamic state";
  Density rho(start=rho_start) "Density - constant, since the fluid is incompressible";
  Power Pf "Fluid-dynamic power";
  Power Pm "Mechanical power";
  MassFlowRate massFlow "Mass flow rate";
  VolumeFlowRate volFlow "Volumetric flow rate";
  Real beta "Pressure ratio";
  PressureDifference dP "Pressure differential";

  replaceable DynTherM.Utilities.PumpCharacteristics.Flow.fixed Flow
    constrainedby DynTherM.Utilities.PumpCharacteristics.BaseClass(
    omega_nom=omega_nom,
    omega=shaft.omega,
    volFlow_nom=volFlow_nom,
    volFlow=volFlow,
    Head_nom=Head_nom) annotation (choicesAllMatching=true);

  replaceable DynTherM.Utilities.PumpCharacteristics.Efficiency.fixed
    Efficiency constrainedby DynTherM.Utilities.PumpCharacteristics.BaseClass(
    omega_nom=omega_nom,
    omega=shaft.omega,
    volFlow_nom=volFlow_nom,
    volFlow=volFlow,
    eta_nom=eta_is) annotation (choicesAllMatching=true);

  CustomInterfaces.Shaft_A shaft annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
  CustomInterfaces.FluidPort_A inlet(
    redeclare package Medium = Medium,
    h_outflow(start=h_start),
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0)) annotation (Placement(transformation(
           extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{
             -90,10}})));
  CustomInterfaces.FluidPort_B outlet(
    redeclare package Medium = Medium,
    h_outflow(start=h_start),
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0)) annotation (Placement(transformation(
           extent={{90,-10},{110,10}}), iconTransformation(extent={{90,-10},{110,
             10}})));

equation
  state_in = Medium.setState_phX(inlet.P, inStream(inlet.h_outflow), inStream(inlet.Xi_outflow));
  rho = Medium.density(state_in);

  // Boundary conditions
  massFlow = inlet.m_flow;
  volFlow = massFlow/rho;

  // Mass balance
  inlet.m_flow + outlet.m_flow = 0;

  // Independent composition mass balances
  inlet.Xi_outflow = inStream(outlet.Xi_outflow);
  outlet.Xi_outflow = inStream(inlet.Xi_outflow);

  // Energy balance
  outlet.h_outflow = inStream(inlet.h_outflow) + Pf/massFlow "Energy balance for w > 0";
  inlet.h_outflow = inStream(outlet.h_outflow) + Pf/massFlow "Energy balance for w < 0";

  // Flow characteristics
  beta = outlet.P/inlet.P;
  dP = outlet.P - inlet.P;
  Flow.Head = dP/rho;
  Pf = dP*volFlow/Efficiency.eta;
  Pm = Pf/eta_m;
  shaft.M*shaft.omega + Pm = 0;

 annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
     Ellipse(extent={{-60,60},{60,-60}}, lineColor={0,0,0},
         lineThickness=0.5),
       Line(points={{-100,0},{-60,0}}, color={0,0,0}),
       Line(points={{60,0},{92,0}}, color={0,0,0}),
       Rectangle(extent={{-4,90},{4,60}}, lineColor={0,0,0}),                                                                                Polygon(origin={-20.716,
              0.7156},                                                                                                            rotation = 180, fillColor={255,255,
              255},
            fillPattern=FillPattern.HorizontalCylinder,                                                                                                    points={{19.284,
              44.7156},{19.284,-43.2844},{-80.716,0.7156},{19.284,44.7156}},
          lineColor={0,0,0},
          lineThickness=0.5)}),                                 Diagram(
       coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The model assumes incompressible flow.</p>
</html>"));
end Pump;
