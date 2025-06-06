within DynTherM.Components.HeatTransfer;
model EnclosedAirSpace "Dynamic model for non-ventilated closed air cavities in double glazing"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  parameter Length w "Width of the air cavity";
  parameter Length h "Height of the air cavity";
  parameter Length l "Length of the air cavity";
  parameter Angle delta "Inclination angle of the air cavity";
  parameter CoefficientOfHeatTransfer ht_start=1
    "Heat transfer coefficient - starting value" annotation (Dialog(tab="Initialization"));

  input Pressure P "Average pressure inside the air cavity";
  input MassFraction X[2] "Mass fraction of the air cavity";

  Area A "Surface of the air cavity";
  Temperature T "Average temperature inside the air cavity";
  NusseltNumber Nu "Nusselt number";
  NusseltNumber Nu_90 "Nusselt number for cavity inclined at 90 degrees";
  NusseltNumber Nu_60 "Nusselt number for cavity inclined at 60 degrees";
  RayleighNumber Ra "Rayleigh number";
  Medium.ThermodynamicState state "Average thermodynamic state";
  CoefficientOfHeatTransfer ht(start=ht_start) "Heat transfer coefficient";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet annotation (
      Placement(transformation(extent={{-14,20},{14,48}}), iconTransformation(
          extent={{-14,20},{14,48}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet annotation (
      Placement(transformation(extent={{-14,-48},{14,-20}}), iconTransformation(
          extent={{-14,-48},{14,-20}})));
protected
  Real G;
  NusseltNumber Nu_1;
  NusseltNumber Nu_2;

equation
  // Thermodynamic state of the air inside the cavity
  state = Medium.setState_pTX(P, T, X);

  // Heat transfer
  A = l*h;
  inlet.Q_flow = ht*A*(inlet.T - outlet.T);
  inlet.Q_flow = - outlet.Q_flow;

  // Compute the heat transfer coefficient
  Nu = ht*w/Medium.thermalConductivity(state);
  Ra = Medium.density(state)^2*w^3*g_n*Medium.specificHeatCapacityCp(
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
    Documentation(info="<html>
<p><b>Assumptions:</b></p>
<ul>
<li> Accumulation&nbsp;of&nbsp;energy,&nbsp;but&nbsp;negligible&nbsp;accumulation&nbsp;of&nbsp;mass&nbsp;and&nbsp;species</li>
<li> Leakage&nbsp;flow&nbsp;is&nbsp;allowed&nbsp;only&nbsp;from&nbsp;inlet&nbsp;to&nbsp;outlet</li>
</ul>
<p><br><b>References:</b></p>
<p>[1] F.&nbsp;Zanghirella&nbsp;et&nbsp;al.&nbsp;&quot;A&nbsp;numerical&nbsp;model&nbsp;to&nbsp;evaluate&nbsp;the&nbsp;thermal&nbsp;behaviour&nbsp;of&nbsp;active&nbsp;transparent&nbsp;facades&quot;,&nbsp;2011.</p>
</html>",
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
