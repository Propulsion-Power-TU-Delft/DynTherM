within DynTherM.Components.HeatTransfer;
model WallConduction "Dynamic model of conduction in a planar surface"
  replaceable model Mat=DynTherM.Materials.Aluminium constrainedby
    DynTherM.Materials.Properties "Material choice" annotation (choicesAllMatching=true);

  // Geometry
  parameter Integer N_cv=1 "Number of control volumes";
  parameter Length t "Wall thickness";
  parameter Area A "Wall surface";

  // Initialization
  parameter Temperature Tstart[N_cv,1]=288.15*ones(N_cv,1)
    "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt "Initialization option" annotation (Dialog(tab="Initialization"));

  final parameter Mass m=Mat.rho*A*t "Mass of the wall";
  final parameter Modelica.Units.SI.HeatCapacity Cm=m*Mat.cm
    "Heat capacity of the wall";

  Temperature T_vol[N_cv,1](start=Tstart) "Average temperature of the wall";

  DynTherM.CustomInterfaces.DistributedHeatPort_A inlet(Nx=N_cv, Ny=1)
    annotation (Placement(transformation(extent={{-40,-12},{40,68}}), iconTransformation(extent={{-40,-12},{40,68}})));
  DynTherM.CustomInterfaces.DistributedHeatPort_B outlet(Nx=N_cv, Ny=1)
    annotation (Placement(transformation(extent={{-40,-68},{40,12}}), iconTransformation(extent={{-40,-68},{40,12}})));

equation
  for i in 1:N_cv loop
    // Energy balance
    Cm/N_cv*der(T_vol[i,1]) = inlet.ports[i,1].Q_flow + outlet.ports[i,1].Q_flow;

    // Heat conduction through the internal half-thickness
    inlet.ports[i,1].Q_flow = (Mat.lambda/N_cv*A*
      (inlet.ports[i,1].T - T_vol[i,1]))/(t/2);

    // Heat conduction through the external half-thickness
    outlet.ports[i,1].Q_flow = (Mat.lambda/N_cv*A*
      (outlet.ports[i,1].T - T_vol[i,1]))/(t/2);
  end for;

initial equation
  for i in 1:N_cv loop
    if initOpt == DynTherM.Choices.InitOpt.steadyState then
      der(T_vol[i,1]) = 0;
    elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
      T_vol[i,1] = Tstart[i,1];
    else
      // do nothing
    end if;
  end for;
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
<p>This is the model of a planar wall. </p>
<p>The heat capacity (which is lumped at the center of the wall thickness) is accounted for, as well as the thermal resistance due to the finite heat conduction coefficient. Longitudinal heat conduction is neglected. </p>
<p>The model can be used to reproduce the heat transfer through many walls in parallel. In that case, the heat flow rate is split equally among the different elements, assuming there is no heat transfer and temperature difference between them.</p>
</html>",
        revisions="<html>
</html>"));
end WallConduction;
