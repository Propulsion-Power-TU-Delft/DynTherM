within DynTherM.Components.HeatTransfer;
model WallConduction "0D Dynamic model of conduction in a planar surface"
  replaceable model Mat=DynTherM.Materials.Aluminium constrainedby
    DynTherM.Materials.Properties          "Material choice" annotation (choicesAllMatching=true);
  parameter Modelica.Units.SI.Length t "Wall thickness";
  parameter Modelica.Units.SI.Area A "Wall surface";
  parameter Modelica.Units.SI.Area A_window=0 "Window area";
  parameter Integer Nw_side=0 "Number of windows per fuselage side";
  parameter Modelica.Units.SI.Temperature Tstart=300
    "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt "Initialization option"
    annotation (Dialog(tab="Initialization"));
  final parameter Modelica.Units.SI.Mass m=Mat.rho*A*t "Mass of the wall";
  final parameter Modelica.Units.SI.HeatCapacity Cm=m*Mat.cm
    "Heat capacity of the wall";
  Modelica.Units.SI.Temperature T_vol(start=Tstart)
    "Average temperature of the wall";
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
    annotation (Placement(transformation(extent={{-14,20},{14,48}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet
    annotation (Placement(transformation(extent={{-14,-48},{14,-20}})));
equation
  Cm*der(T_vol) = inlet.Q_flow + outlet.Q_flow "Energy balance";
  inlet.Q_flow = (Mat.lambda*(A - A_window)*(inlet.T - T_vol))/(t/2)
    "Heat conduction through the internal half-thickness";
  outlet.Q_flow = (Mat.lambda*(A - A_window)*(outlet.T - T_vol))/(t/2)
    "Heat conduction through the external half-thickness";
initial equation
  if initOpt == DynTherM.Choices.InitOpt.steadyState then
    der(T_vol) = 0;
  elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
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
          textString="WALL")}),
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
end WallConduction;
