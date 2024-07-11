within DynTherM.Components.HeatTransfer;
model ConductionPlanoConcave2D
  "Dynamic model of conduction heat transfer through a Plano-Concave surface, where the concave side is one-fourth of a circle "

  replaceable model Mat=Materials.Aluminium constrainedby Materials.Properties
                         "Material choice" annotation (choicesAllMatching=true);

  input Real N = 1;
  input Length R "Radius of curvature of concave side" annotation (Dialog(enable=true));
  input Length y "Distance from the center of of curvature of concave side to the side opposite to the concave side" annotation (Dialog(enable=true));
  input Length dz "Out of plane thickness" annotation (Dialog(enable=true));

  // Initialization
  parameter Temperature Tstart=300
    "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  Length t_avg "Equivalent thickness if it was a rectangle";
  Length h "Height, In-plane dimension perpendicular to thickness";
  Area As "Plane area";

  Mass m "Mass of the wall";
  Modelica.Units.SI.HeatCapacity Cm "Heat capacity of the wall";
  Temperature T_vol "Average temperature of the wall";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inletPlanar
    "Thermal port for the planar side" annotation (Placement(transformation(
          extent={{-10,80},{10,100}}),iconTransformation(extent={{-10,80},{10,100}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b OutletOppoCon
    "Thermal port for the side opposite to concave side" annotation (Placement(
        transformation(extent={{-60,-10},{-40,10}}),  iconTransformation(extent={{-60,-10},
            {-40,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outletPlanar
    "Thermal port for the planar side" annotation (Placement(transformation(extent={{-10,
            -100},{10,-80}}), iconTransformation(extent={{-10,-100},{10,-80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a InletConcave
    "Thermal port for the concave side" annotation (Placement(transformation(
          extent={{12,-10},{32,10}}),  iconTransformation(extent={{12,-10},{32,10}})));

equation
  t_avg = y/2 - R*(pi+2)/(8*sqrt(2))     "Areas integration";
  As = y*R*sqrt(2) - R^2 * (pi+2)/4      "Using geometry";
  h = R*sqrt(2)                          "Using geometry";


  m=Mat.rho*As*dz;
  Cm=m*Mat.cm;

  N*Cm*der(T_vol) =inletPlanar.Q_flow + outletPlanar.Q_flow + InletConcave.Q_flow
     + OutletOppoCon.Q_flow                                                         "Energy balance";
  inletPlanar.Q_flow = (Mat.lambda*N*dz*(R/sqrt(2))*(inletPlanar.T - T_vol))/(h/2)
    "Heat conduction through the upper half of planar side";
  outletPlanar.Q_flow = (Mat.lambda*N*dz*(R/sqrt(2))*(outletPlanar.T - T_vol))/(h/2)
    "Heat conduction through the lower half of planar side";
  InletConcave.Q_flow = (Mat.lambda*N*dz*(pi*R/2)*(InletConcave.T - T_vol))/(t_avg/2)
    "Heat conduction through the concave side";
  OutletOppoCon.Q_flow = (Mat.lambda*N*dz*h*(OutletOppoCon.T - T_vol))/(t_avg/2)
    "Heat conduction through planar side opposite to the concave";


initial equation
  if initOpt == Choices.InitOpt.steadyState then
    der(T_vol) = 0;
  elseif initOpt == Choices.InitOpt.fixedState then
    T_vol = Tstart;
  else
    // do nothing
  end if;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Polygon(
          points={{40,80},{32,60},{26,40},{22,22},{20,0},{22,-20},{26,-40},{34,-60},
              {42,-80},{40,-80},{-40,-80},{-40,80},{40,80}},
          lineColor={28,108,200},
          fillColor={215,215,215},
          fillPattern=FillPattern.Backward)}),                   Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ConductionPlanoConcave2D;
