within DynTherM.Components.HeatTransfer;
model WallConduction2D
  "Dynamic model of conduction in a planar surface"
  replaceable model Mat=Materials.Aluminium constrainedby Materials.Properties
                         "Material choice" annotation (choicesAllMatching=true);

  input Real N=1 "Number of walls in parallel" annotation (Dialog(enable=true));
  input Length t "Wall thickness" annotation (Dialog(enable=true));
  input Length h "Wall height, dimension perpendicular to thickness along heat transfer" annotation (Dialog(enable=true));
  input Area A "Wall surface" annotation (Dialog(enable=true));

  // Initialization
  parameter Temperature Tstart=300
    "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  Mass m "Mass of the wall";
  Modelica.Units.SI.HeatCapacity Cm "Heat capacity of the wall";
  Temperature T_vol "Average temperature of the wall";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
    annotation (Placement(transformation(extent={{-14,22},{14,50}}),
        iconTransformation(extent={{-14,22},{14,50}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outletH
    annotation (Placement(transformation(extent={{-100,-14},{-72,14}}),
        iconTransformation(extent={{-100,-14},{-72,14}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet
    annotation (Placement(transformation(extent={{-14,-48},{14,-20}})));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inletH
    annotation (Placement(transformation(extent={{72,-12},{100,16}}),
        iconTransformation(extent={{72,-12},{100,16}})));

equation

  m=Mat.rho*A*t;
  Cm=m*Mat.cm;

  N*Cm*der(T_vol) = inlet.Q_flow + outlet.Q_flow +  inletH.Q_flow + outletH.Q_flow  "Energy balance";
  inlet.Q_flow = (Mat.lambda*N*A*(inlet.T - T_vol))/(t/2)
    "Heat conduction through the internal half-thickness";
  outlet.Q_flow = (Mat.lambda*N*A*(outlet.T - T_vol))/(t/2)
    "Heat conduction through the external half-thickness";
  inletH.Q_flow = (Mat.lambda*N*A*(inletH.T - T_vol))/(h/2)
    "Vertical heat conduction through the upper half-height";
  outletH.Q_flow = (Mat.lambda*N*A*(outletH.T - T_vol))/(h/2)
    "Vertical heat conduction through the lower half-height";


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
          extent={{-80,22},{80,-20}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Text(
          extent={{-32,30},{32,-34}},
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
end WallConduction2D;
