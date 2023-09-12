within ThermalManagement.Components.HeatTransfer;
model EnclosedAirSpace
  "0D Dynamic model for non-ventilated closed air cavities in double glazing"
  // Hp: accumulation of energy, but negligible accumulation of mass and species
  // Hp: leakage flow is allowed only from inlet to outlet
  // References
  // F. Zanghirella et al. - A numerical model to evaluate the thermal behaviour of active transparent facades, 2011.

  package Medium = Modelica.Media.Air.MoistAir;
  outer ThermalManagement.Components.Environment environment "Environmental properties";
  parameter ThermalManagement.Choices.InitOpt initOpt=environment.initOpt
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Units.SI.Length w "Width of the air cavity";
  parameter Modelica.Units.SI.Length h "Height of the air cavity";
  parameter Modelica.Units.SI.Length l "Length of the air cavity";
  parameter Modelica.Units.SI.Angle delta "Inclination angle of the air cavity";
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer ht_start=1
    "Heat transfer coefficient - starting value" annotation (Dialog(tab="Initialization"));

  input Modelica.Units.SI.Pressure P "Average pressure inside the air cavity";
  input Modelica.Units.SI.MassFraction X[2] "Mass fraction of the air cavity";

  Modelica.Units.SI.Area A "Surface of the air cavity";
  Modelica.Units.SI.Temperature T "Average temperature inside the air cavity";
  Modelica.Units.SI.NusseltNumber Nu "Nusselt number";
  Modelica.Units.SI.NusseltNumber Nu_90 "Nusselt number for cavity inclined at 90 degrees";
  Modelica.Units.SI.NusseltNumber Nu_60 "Nusselt number for cavity inclined at 60 degrees";
  Modelica.Units.SI.RayleighNumber Ra "Rayleigh number";
  Medium.ThermodynamicState state "Average thermodynamic state";
  Modelica.Units.SI.CoefficientOfHeatTransfer ht(start=ht_start)
    "Heat transfer coefficient";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet annotation (
      Placement(transformation(extent={{-14,20},{14,48}}), iconTransformation(
          extent={{-14,20},{14,48}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet annotation (
      Placement(transformation(extent={{-14,-48},{14,-20}}), iconTransformation(
          extent={{-14,-48},{14,-20}})));
protected
  Real G;
  Modelica.Units.SI.NusseltNumber Nu_1;
  Modelica.Units.SI.NusseltNumber Nu_2;

equation
  // Thermodynamic state of the air inside the cavity
  state = Medium.setState_pTX(P, T, X);

  // Heat transfer
  A =l*h;
  inlet.Q_flow = ht*A*(inlet.T - outlet.T);
  inlet.Q_flow = - outlet.Q_flow;

  // Compute the heat transfer coefficient
  Nu = ht*w/Medium.thermalConductivity(state);
  Ra = Medium.density(state)^2*w^3*environment.g*Medium.specificHeatCapacityCp(
    state)*abs(inlet.T - outlet.T)/(Medium.dynamicViscosity(state)*
    Medium.thermalConductivity(state)*T);

  // Evaluate Nusselt number for cavity inclined at 90 degrees
  if Ra > 5e4 then
    Nu_90 = 0.0673838*Ra^(1/3);
  elseif (Ra > 1e4) and (Ra <= 5e4) then
    Nu_90 = 0.028154*Ra^0.4134;
  else
    Nu_90 = 1 + 1.75967*1e-10*Ra^2.2984755;
  end if;

  // Evaluate Nusselt number for cavity inclined at 60 degrees
  G = 0.5/(1 + (Ra/3160)^20.6)^0.1;
  Nu_1 = (1 + (0.0936*Ra^0.314/(1 + G))^7)^(1/7);
  Nu_2 = (0.104 + 0.175*w/h)*Ra^0.283;
  Nu_60 = max(Nu_1, Nu_2);

  // Evaluate the Nusselt number as a function of the inclination angle
  if ((Modelica.Units.Conversions.to_deg(delta) >= 0) and (Modelica.Units.Conversions.to_deg(delta) < 60)) then
    Nu = 1 + 1.44*(abs(1 - 1708/(Ra*cos(delta))) +
      1 - 1708/(Ra*cos(delta)))/2*
      (1 - sin(1.8*delta)^1.6*1708/(Ra*cos(delta))) +
      (abs((Ra*cos(delta)/5830)^(1/3) - 1) + (Ra*cos(delta)/5830)^(1/3) - 1)/2;
  elseif (Modelica.Units.Conversions.to_deg(delta) == 60) then
    Nu = Nu_60;
  elseif ((Modelica.Units.Conversions.to_deg(delta) > 60) and (Modelica.Units.Conversions.to_deg(delta) < 90)) then
    Nu = Nu_60 + (Modelica.Units.Conversions.to_deg(delta) - 60)*(Nu_90 - Nu_60)/30;
  elseif (Modelica.Units.Conversions.to_deg(delta) == 90) then
    Nu = Nu_90;
  end if;

  // Boundary conditions
  T = (inlet.T + outlet.T)/2;

  assert(Modelica.Units.Conversions.to_deg(delta) <= 90, "The inclination angle is outside the range 0-90 degrees");

  annotation (
    Icon(graphics={   Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,0}),
        Text(
          extent={{-38,42},{46,-42}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="AIR GAP")}),
    Documentation(info="<HTML>
<p>This is the model of a cylindrical tube of solid material.
<p>The heat capacity (which is lumped at the center of the tube thickness) is accounted for, as well as the thermal resistance due to the finite heat conduction coefficient. Longitudinal heat conduction is neglected.
<p><b>Modelling options</b></p>
<p>The following options are available:
<ul>
<li><tt>WallRes = false</tt>: the thermal resistance of the tube wall is neglected.
<li><tt>WallRes = true</tt>: the thermal resistance of the tube wall is accounted for.
</ul>
</HTML>",
        revisions="<html>
<ul>
<li><i>30 May 2005</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Initialisation support added.</li>
<li><i>1 Oct 2003</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       First release.</li>
</ul>
</html>
"));
end EnclosedAirSpace;
