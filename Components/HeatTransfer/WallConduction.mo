within DynTherM.Components.HeatTransfer;
model WallConduction "Dynamic model of conduction in a planar surface"
  replaceable model Mat=Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);

  input Length t "Wall thickness" annotation (Dialog(enable=true));
  input Area A "Wall surface" annotation (Dialog(enable=true));

  // Initialization
  parameter Temperature Tstart=298.15
    "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  Mass m "Mass of the wall";
  Modelica.Units.SI.HeatCapacity Cm "Heat capacity of the wall";
  Temperature T_vol "Average temperature of the wall";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
    annotation (Placement(transformation(extent={{-14,20},{14,48}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet
    annotation (Placement(transformation(extent={{-14,-48},{14,-20}})));

equation
  m = Mat.rho*A*t;
  Cm = m*Mat.cm;

  Cm*der(T_vol) = inlet.Q_flow + outlet.Q_flow "Energy balance";
  inlet.Q_flow = (Mat.lambda*A*(inlet.T - T_vol))/(t/2)
    "Heat conduction through the internal half-thickness";
  outlet.Q_flow = (Mat.lambda*A*(outlet.T - T_vol))/(t/2)
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
          textString="WALL")}),
    Documentation(info="<html>
<p>The heat capacity (which is lumped at the center of the wall thickness) is accounted for, as well as the thermal resistance due to the finite heat conduction coefficient. Longitudinal heat conduction is neglected. </p>
<p>The model can be used to reproduce the heat transfer through many walls in parallel. In that case, the heat flow rate is split equally among the different elements, assuming there is no heat transfer and temperature difference between them.</p>
<p>Model adapted from ThermoPower library by Francesco Casella.</p>
</html>",
        revisions="<html>
</html>"));
end WallConduction;
