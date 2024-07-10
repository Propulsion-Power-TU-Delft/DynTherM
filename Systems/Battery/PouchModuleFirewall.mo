within DynTherM.Systems.Battery;
model PouchModuleFirewall
  "Battery module made by stacking pouch cells in parallel and by adding internal firewall between adjacent cells"

  replaceable model InPlaneCellMat = Materials.PolestarCellInPlane constrainedby
    Materials.Properties "In-plane cell material properties" annotation (choicesAllMatching=true);

  replaceable model CrossPlaneCellMat = Materials.PolestarCellCrossPlane constrainedby
    Materials.Properties "Cross-plane cell material properties" annotation (choicesAllMatching=true);

  replaceable model FirewallMat = Materials.PolyurethaneFoam constrainedby
    Materials.Properties "Firewall material" annotation (choicesAllMatching=true);

  model Cell = Components.Battery.PouchCell1D "Cell";
  model Firewall = Components.OneDimensional.WallConduction1D "Firewall";

  // Geometry
  parameter Length W_cell "Cell width" annotation (Dialog(tab="Geometry"));
  parameter Length H_cell "Cell height" annotation (Dialog(tab="Geometry"));
  parameter Length t_cell "Cell thickness" annotation (Dialog(tab="Geometry"));
  parameter Length t_fw "Firewall thickness" annotation (Dialog(tab="Geometry"));

  // Electrical parameters
  parameter Real eta=0.98 "Cell charging/discharging efficiency";
  parameter ElectricCharge C_nom "Nominal cell capacity";

  // Initialization
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Real SoC_start "Starting state of charge" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart=298.15 "Temperature start value" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N_cv(min=1)=10 "Number of vertical control volumes in which each cell is discretized";
  parameter Integer N_parallel(min=1) "Number of cells electrically connected in parallel";
  parameter Integer N_series(min=1) "Number of cells electrically connected in series";
  final parameter Integer N_cells=N_parallel*N_series "Number of cells stacked in parallel";

  Cell cell[N_cells](
    redeclare model InPlaneMat=InPlaneCellMat,
    redeclare model CrossPlaneMat=CrossPlaneCellMat,
    each W=W_cell,
    each H=H_cell,
    each t=t_cell,
    each eta=eta,
    each C_nom=C_nom,
    each SoC_start=SoC_start,
    each Tstart=Tstart,
    each initOpt=initOpt,
    each N=N_cv,
    each I=I/N_parallel);

  Firewall firewall[N_cells-1](
    redeclare model Mat=FirewallMat,
    each A=W_cell*H_cell,
    each t=t_fw,
    each Tstart=Tstart,
    each initOpt=initOpt,
    each N=N_cv);

  Length t_module "Module thickness";

  CustomInterfaces.DistributedHeatPort_A Bottom(Nx=N_cells, Ny=1) annotation (
      Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={0,-40}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=0,
        origin={-20,-66})));
  CustomInterfaces.DistributedHeatPort_A Top(Nx=N_cells, Ny=1) annotation (Placement(
        transformation(
        extent={{-12.5,-12.5},{12.5,12.5}},
        rotation=180,
        origin={0.5,40.5}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=0,
        origin={-20,26})));
  CustomInterfaces.DistributedHeatPort_A Left(Nx=N_cv, Ny=1) annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={-40,0}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=-90,
        origin={-76,-20})));
  CustomInterfaces.DistributedHeatPort_A Right(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={40,0}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=-90,
        origin={36,-20})));
  Modelica.Blocks.Interfaces.RealInput I "Current - positive if charging" annotation (Placement(transformation(
          extent={{-116,-38},{-96,-18}}), iconTransformation(
        extent={{8,-8},{-8,8}},
        rotation=90,
        origin={-8,48})));
  Modelica.Blocks.Interfaces.RealOutput V "Voltage drop" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={102,-38}), iconTransformation(
        extent={{-8,-8},{8,8}},
        rotation=90,
        origin={8,48})));

equation
  V = cell[1].V*N_series;
  t_module = N_cells*t_cell + (N_cells - 1)*t_fw;

  // External ports connections
  for i in 1:N_cells loop
    connect(Top.ports[i,1], cell[i].Top);
    connect(Bottom.ports[i,1], cell[i].Bottom);
  end for;

  connect(Left, cell[1].Left);
  connect(Right, cell[end].Right);

  // Internal ports connections
  for i in 1:(N_cells-1) loop
    connect(cell[i].Right, firewall[i].inlet);
    connect(firewall[i].outlet, cell[i+1].Left);
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-70,20},{-50,-60}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-64,-4},{-56,-36}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(points={{-70,20},{-30,60},{-10,60},{-50,20}}, color={0,0,0}),
        Line(points={{-10,60},{-10,-20},{-50,-60}}, color={0,0,0}),
        Rectangle(
          extent={{-30,20},{-10,-60}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-24,-4},{-16,-36}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(points={{-30,20},{10,60},{30,60},{-10,20}}, color={0,0,0}),
        Line(points={{30,60},{30,-20},{-10,-60}}, color={0,0,0}),
        Rectangle(
          extent={{10,20},{30,-60}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{16,-4},{24,-36}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(points={{10,20},{50,60},{70,60},{30,20}}, color={0,0,0}),
        Line(points={{70,60},{70,-20},{30,-60}}, color={0,0,0})}),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Pouch cells are physically stacked in parallel, but they might be electrically connected both in series and parallel.</p>
</html>"));
end PouchModuleFirewall;
