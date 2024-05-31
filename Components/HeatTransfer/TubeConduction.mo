within DynTherM.Components.HeatTransfer;
model TubeConduction "Dynamic model of conduction in a hollow cylinder"

  replaceable model Mat=Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);

  parameter Integer N=1 "Number of tubes in parallel";
  parameter Real coeff "Fraction of tube with active heat transfer";
  parameter Length L "Tube length";
  parameter Length R_ext "Tube external radius";
  parameter Length R_int "Tube internal radius";

  parameter Length L_window=0 "Window length - aircraft fuselage application" annotation (Dialog(tab="Passive surface"));
  parameter Length H_window=0 "Window height - aircraft fuselage application" annotation (Dialog(tab="Passive surface"));
  parameter Integer Nw_side=0 "Number of windows per fuselage side - aircraft fuselage application" annotation (Dialog(tab="Passive surface"));

  // Initialization
  parameter Temperature Tstart=300
    "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  final parameter Mass m=coeff*Mat.rho*L*pi*(R_ext^2 - R_int^2) "Mass of the tube";
  final parameter Modelica.Units.SI.HeatCapacity Cm=m*Mat.cm
    "Heat capacity of the tube";

  Temperature T_vol "Average temperature of the tube";
  Length A_window_int
    "Equivalent internal window area - aircraft fuselage application";
  Length A_window_ext
    "Equivalent external window area - aircraft fuselage application";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
    annotation (Placement(transformation(extent={{-14,20},{14,48}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet
    annotation (Placement(transformation(extent={{-14,-48},{14,-20}})));

equation
  assert(R_ext > R_int, "External radius must be greater than internal radius");

  A_window_int = H_window/R_int*L_window*Nw_side;
  A_window_ext = H_window/R_ext*L_window*Nw_side;

  N*Cm*der(T_vol) = inlet.Q_flow + outlet.Q_flow "Energy balance";
  inlet.Q_flow = (Mat.lambda*N*(coeff*2*pi*L - A_window_int)*(inlet.T - T_vol))/
    Modelica.Math.log((R_int + R_ext)/(2*R_int))
    "Heat conduction through the internal half-thickness";
  outlet.Q_flow = (Mat.lambda*N*(coeff*2*pi*L - A_window_ext)*(outlet.T - T_vol))/
    Modelica.Math.log((2*R_ext)/(R_int + R_ext))
    "Heat conduction through the external half-thickness";

initial equation
  if initOpt == Choices.InitOpt.steadyState then
    der(T_vol) = 0;
  elseif initOpt == Choices.InitOpt.fixedState then
    T_vol = Tstart;
  else
    // do nothing
  end if;
  annotation (
    Icon(graphics={   Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Text(
          extent={{-30,32},{34,-32}},
          lineColor={255,255,255},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="TUBE")}),
    Documentation(info="<html>
<p>The heat capacity (which is lumped at the center of the tube thickness) is accounted for, as well as the thermal resistance due to the finite heat conduction coefficient. Longitudinal heat conduction is neglected.</p>
<p>The model can be used to reproduce the heat transfer through many tubes in parallel. In that case, the heat flow rate is split equally among the different tubes, assuming there is no heat transfer and temperature difference between them.</p>
<p>The tube element can be used to model the fuselage of an aircraft. In that case, the heat transfer through the cabin windows is neglected and treated separately.</p>
<p>Model adapted from ThermoPower library by Francesco Casella.</p>
</html>",
        revisions="<html>
</html>"));
end TubeConduction;
