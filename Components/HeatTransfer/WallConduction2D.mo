within DynTherM.Components.HeatTransfer;
model WallConduction2D
  "Dynamic model of conduction in a planar surface"
  replaceable model Mat=Materials.Aluminium constrainedby Materials.Properties
                         "Material choice" annotation (choicesAllMatching=true);

  input Real N=1 "Number of walls in parallel" annotation (Dialog(enable=true));
  input Length w "Width of the Plane, sides facing east and west"  annotation (Dialog(enable=true));
  input Length l "Length of the plane,sides facing north and south" annotation (Dialog(enable=true));
  input Length dz "Thickness of the slice, out of plane" annotation (Dialog(enable=true));

  // Initialization
  parameter Temperature Tstart=300
    "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  Mass m "Mass of the volume";
  Modelica.Units.SI.HeatCapacity Cm "Heat capacity of the volume";
  Temperature T_vol "Average temperature of the volume";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inletN annotation (
      Placement(transformation(extent={{-10,60},{12,82}}), iconTransformation(
          extent={{-10,60},{12,82}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outletW
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}}),
        iconTransformation(extent={{-100,-10},{-80,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outletS
    annotation (Placement(transformation(extent={{-12,-82},{10,-60}}),
        iconTransformation(extent={{-12,-82},{10,-60}})));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inletE
    annotation (Placement(transformation(extent={{80,-10},{100,10}}),
        iconTransformation(extent={{80,-10},{100,10}})));

equation

  m=Mat.rho*w*l*dz;
  Cm=m*Mat.cm;

  N*Cm*der(T_vol) =inletN.Q_flow +outletS.Q_flow  +inletE.Q_flow  +outletW.Q_flow         "Energy balance";

  inletN.Q_flow = (Mat.lambda*N*w*dz*(inletN.T - T_vol))/(l/2)
    "Heat conduction through north side";
  outletS.Q_flow = (Mat.lambda*N*w*dz*(outletS.T - T_vol))/(l/2)
    "Heat conduction through the southern side";
  inletE.Q_flow = (Mat.lambda*N*l*dz*(inletE.T - T_vol))/(w/2)
    "Heat conduction through the eastern side";
  outletW.Q_flow = (Mat.lambda*N*l*dz*(outletW.T - T_vol))/(w/2)
    "Heat conduction through the Western side";


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
          extent={{-80,60},{80,-60}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Text(
          extent={{-32,20},{32,-44}},
          lineColor={255,255,255},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Plane
"),
        Line(
          points={{-2,-34},{-2,36}},
          color={0,0,0},
          arrow={Arrow.None,Arrow.Filled},
          thickness=0.5)}),
    Documentation(info="<html>
<p>The heat capacity (which is lumped at the center of the wall thickness) is accounted for, as well as the thermal resistance due to the finite heat conduction coefficient. Longitudinal heat conduction is neglected. </p>
<p>The model can be used to reproduce the heat transfer through many walls in parallel. In that case, the heat flow rate is split equally among the different elements, assuming there is no heat transfer and temperature difference between them.</p>
<p>Model adapted from ThermoPower library by Francesco Casella.</p>
</html>",
        revisions="<html>
</html>"));
end WallConduction2D;
