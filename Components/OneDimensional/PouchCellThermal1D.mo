within DynTherM.Components.OneDimensional;
model PouchCellThermal1D
  "Thermal model of a pouch cell featuring 1D discretization"

  replaceable model Mat = Materials.PolestarCell constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  model CV_cell = Components.OneDimensional.PouchCellThermalCV
    "Control volume for cell";

  // Geometry
  parameter Area A "Base surface area" annotation (Dialog(tab="Geometry"));
  parameter Length H "Height" annotation (Dialog(tab="Geometry"));

  // Initialization
  parameter Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt  "Initialization option" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N(min=1) "Number of vertical sections in which the cell is discretized";

  CV_cell Cell[N](
    redeclare model Mat=Mat,
    each H=H/N,
    each A=A,
    each Tstart=Tstart,
    each initOpt=initOpt,
    each Q_gen=Average.Q_flow/N);

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b Top annotation (Placement(
        transformation(extent={{28,30},{48,50}}), iconTransformation(extent={{-10,
            54},{6,70}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Average annotation (
      Placement(transformation(extent={{46,-24},{66,-4}}), iconTransformation(
          extent={{-10,-8},{6,8}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b Bottom annotation (
      Placement(transformation(extent={{4,-76},{24,-56}}), iconTransformation(
          extent={{-10,-70},{6,-54}})));

equation
  Average.T = sum(Cell.T_vol)/N;

  // Internal connections
  for i in 1:(N-1) loop
    connect(Cell[i].outlet, Cell[i+1].inlet);
  end for;

  // Boundary connections
  connect(Cell[1].inlet, Bottom);
  connect(Cell[end].outlet, Top);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Bitmap(
          extent={{-88,-88},{88,88}},
          fileName="modelica://DynTherM/Figures/PouchCell.PNG",
          origin={6,0},
          rotation=-90),
        Line(
          points={{0,24},{0,-100}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-52,-26},
          rotation=90),
        Line(
          points={{0,24},{0,-100}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-52,0},
          rotation=90),
        Line(
          points={{0,24},{0,-100}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-52,26},
          rotation=90)}),                      Diagram(coordinateSystem(
          preserveAspectRatio=false)),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PouchCellThermal1D;
