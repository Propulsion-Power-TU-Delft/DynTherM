within DynTherM.Systems.Battery;
model PouchModuleParallel "Battery module made of pouch cells"
  // USE WALLCONDUCTION1D FOR RESIN AND FRAME!!!

  replaceable model InPlaneCellMat = Materials.PolestarCellInPlane constrainedby
    Materials.Properties "In-plane cell material properties" annotation (choicesAllMatching=true);

  replaceable model CrossPlaneCellMat = Materials.PolestarCellCrossPlane constrainedby
    Materials.Properties "Cross-plane cell material properties" annotation (choicesAllMatching=true);

  replaceable model FirewallMat = Materials.CompressionPadFoam constrainedby
    Materials.Properties "Firewall material" annotation (choicesAllMatching=true);

  replaceable model ResinMat = Materials.ThermalResin constrainedby
    Materials.Properties "Resin material" annotation (choicesAllMatching=true);

  replaceable model FrameMat = Materials.AluminiumFoil constrainedby
    Materials.Properties "Frame material" annotation (choicesAllMatching=true);

  model Cell = Components.Electrical.PouchCell1D "Cell";
  model Firewall = Components.OneDimensional.WallConduction1D "Firewall in between adjacent cells";
  model Resin = Components.HeatTransfer.WallConduction "Thermal resin";
  model Frame = Components.HeatTransfer.WallConduction "Frame/casing";

  // Geometry
  parameter Length W_cell "Cell width" annotation (Dialog(tab="Geometry"));
  parameter Length H_cell "Cell height" annotation (Dialog(tab="Geometry"));
  parameter Length t_cell "Cell thickness" annotation (Dialog(tab="Geometry"));
  parameter Length t_fw "Thickness of firewall between cells in parallel" annotation (Dialog(tab="Geometry"));
  parameter Length t_resin "Thickness of resin between cells and frame" annotation (Dialog(tab="Geometry"));
  parameter Length t_frame "Frame thickness" annotation (Dialog(tab="Geometry"));

  // Electrical parameters
  parameter Real eta=0.98 "Cell charging/discharging efficiency";
  parameter ElectricCharge C_nom "Nominal cell capacity";

  // Initialization
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Real SoC_start "State of charge - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart=298.15 "Temperature - start value" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N_cv(min=1)=10 "Number of vertical control volumes in which each cell is discretized";
  parameter Integer Ns(min=1) "Number of cells connected in series";
  parameter Integer Np(min=1) "Number of cells connected in parallel";

  Length H_module "Module height";
  Length t_module "Module thickness";

  Cell cell[Ns,Np](
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
    each N=N_cv);

  Firewall firewall[Ns*Np - 1](
    redeclare model Mat=FirewallMat,
    each A=W_cell*H_cell,
    each t=t_fw,
    each Tstart=Tstart,
    each initOpt=initOpt,
    each N=N_cv);

  Resin resin_top[Ns*Np](
    redeclare model Mat=ResinMat,
    each A=W_cell*(t_cell + t_fw/2),
    each t=t_resin,
    each Tstart=Tstart,
    each initOpt=initOpt);

  Resin resin_bottom[Ns*Np](
    redeclare model Mat=ResinMat,
    each A=W_cell*(t_cell + t_fw/2),
    each t=t_resin,
    each Tstart=Tstart,
    each initOpt=initOpt);

  Frame frame_top[Ns*Np](
    redeclare model Mat=FrameMat,
    each A=W_cell*(t_cell + t_fw/2),
    each t=t_frame,
    each Tstart=Tstart,
    each initOpt=initOpt);

  Frame frame_bottom[Ns*Np](
    redeclare model Mat=FrameMat,
    each A=W_cell*(t_cell + t_fw/2),
    each t=t_frame,
    each Tstart=Tstart,
    each initOpt=initOpt);

  Frame frame_left[N_cv](
    redeclare model Mat=FrameMat,
    each A=W_cell*H_cell,
    each t=t_frame,
    each Tstart=Tstart,
    each initOpt=initOpt);

  Frame frame_right[N_cv](
    redeclare model Mat=FrameMat,
    each A=W_cell*H_cell,
    each t=t_frame,
    each Tstart=Tstart,
    each initOpt=initOpt);

  CustomInterfaces.DistributedHeatPort_A Bottom(Nx=Ns*Np, Ny=1)   annotation (
      Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={0,-38}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=0,
        origin={-20,-66})));
  CustomInterfaces.DistributedHeatPort_A Top(Nx=Ns*Np, Ny=1)   annotation (Placement(
        transformation(
        extent={{-12.5,-12.5},{12.5,12.5}},
        rotation=180,
        origin={0.5,40.5}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=0,
        origin={-20,26})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p annotation (Placement(
        transformation(extent={{-108,-8},{-92,8}}),    iconTransformation(
          extent={{-66,-26},{-54,-14}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n annotation (Placement(
        transformation(extent={{108,-8},{92,8}}),     iconTransformation(extent={{14,-26},
            {26,-14}})));

  Modelica.Electrical.Analog.Sensors.MultiSensor multiSensor
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Modelica.Electrical.Batteries.Interfaces.CellBus batteryBus
    "Battery bus (average / sum over all cells)" annotation (Placement(
        transformation(extent={{-68,-70},{-32,-34}}), iconTransformation(extent={{-10,30},
            {10,50}})));

  CustomInterfaces.DistributedHeatPort_A Left(Nx=N_cv, Ny=1) annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={-40,40}),iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=-90,
        origin={-76,-20})));
  CustomInterfaces.DistributedHeatPort_A Right(Nx=N_cv, Ny=1) annotation (
      Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={40,40}),iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=-90,
        origin={36,-20})));

equation
  // Geometry
  H_module = H_cell + 2*t_resin + 2*t_frame;
  t_module = Np*Ns*t_cell + (Np*Ns - 1)*t_fw + 2*t_frame;

  // -------------------------------  ELECTRICAL -------------------------------
  // External electrical ports connections
  connect(multiSensor.nc, cell[1,1].p);
  connect(cell[Ns,1].n, n);

  // Internal electrical ports connections
  // Parallel
  for ks in 1:Ns loop
    for kp in 1:(Np - 1) loop
      connect(cell[ks,kp].p, cell[ks,kp + 1].p);
      connect(cell[ks,kp].n, cell[ks,kp + 1].n);
    end for;
  end for;

  // Series
  for ks in 1:(Ns - 1) loop
    connect(cell[ks,1].n, cell[ks + 1,1].p);
  end for;

  // --------------------------------  THERMAL ---------------------------------
  // External thermal ports connections
  for ks in 1:Ns loop
    for kp in 1:Np loop
      connect(cell[ks,kp].Top, resin_top[Np*(ks - 1) + kp].inlet);
      connect(resin_top[Np*(ks - 1) + kp].outlet, frame_top[Np*(ks - 1) + kp].inlet);
      connect(frame_top[Np*(ks - 1) + kp].outlet, Top.ports[Np*(ks - 1) + kp,1]);
      connect(cell[ks,kp].Bottom, resin_bottom[Np*(ks - 1) + kp].inlet);
      connect(resin_bottom[Np*(ks - 1) + kp].outlet, frame_bottom[Np*(ks - 1) + kp].inlet);
      connect(frame_bottom[Np*(ks - 1) + kp].outlet, Bottom.ports[Np*(ks - 1) + kp,1]);
    end for;
  end for;

  connect(Left, cell[1,1].Left);
  connect(Right, cell[Ns,Np].Right);

  // Internal thermal ports connections
  for ks in 1:Ns loop
    for kp in 1:(Np - 1) loop
      connect(cell[ks,kp].Right, firewall[Np*(ks - 1) + kp].inlet);
      connect(firewall[Np*(ks - 1) + kp].outlet, cell[ks,kp + 1].Left);
    end for;
  end for;

  for ks in 1:(Ns - 1) loop
    connect(cell[ks,Np].Right, firewall[Np*(ks - 1) + Np].inlet);
    connect(firewall[Np*(ks - 1) + Np].outlet, cell[ks + 1,1].Left);
  end for;

  connect(p, multiSensor.pc)
    annotation (Line(points={{-100,0},{-60,0}}, color={0,0,255}));
  connect(multiSensor.pc, multiSensor.pv)
    annotation (Line(points={{-60,0},{-60,10},{-50,10}}, color={0,0,255}));
  connect(n, multiSensor.nv) annotation (Line(points={{100,0},{100,-20},{-50,
          -20},{-50,-10}},
                      color={0,0,255}));
  connect(multiSensor.v, batteryBus.v) annotation (Line(points={{-44,-11},{-44,
          -52},{-50,-52},{-50,-51.91},{-49.91,-51.91}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(multiSensor.i, batteryBus.i) annotation (Line(points={{-56,-11},{-56,
          -51.91},{-49.91,-51.91}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(multiSensor.power, batteryBus.power) annotation (Line(points={{-61,-6},
          {-70,-6},{-70,-51.91},{-49.91,-51.91}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
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
<p>Pouch cells are electrically connected in series/parallel, see image below.</p>
<p>However, they are physically stacked in parallel.</p>
<p><img src=\"modelica://DynTherM/Figures/pouch_cell_module.png\"/></p>
</html>"));
end PouchModuleParallel;
