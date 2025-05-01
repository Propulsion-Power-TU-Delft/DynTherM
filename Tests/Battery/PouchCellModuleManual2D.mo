within DynTherM.Tests.Battery;
model PouchCellModuleManual2D
  replaceable model InPlaneCellMat = Materials.PolestarCellInPlane constrainedby
    Materials.Properties "In-plane cell material properties" annotation (choicesAllMatching=true);

  replaceable model CrossPlaneCellMat = Materials.PolestarCellCrossPlane constrainedby
    Materials.Properties "Cross-plane cell material properties" annotation (choicesAllMatching=true);

  replaceable model FirewallMat = Materials.PolyurethaneFoam constrainedby
    Materials.Properties "Firewall material" annotation (choicesAllMatching=true);

  replaceable model ResinMat = Materials.ThermalResin constrainedby
    Materials.Properties "Resin material" annotation (choicesAllMatching=true);

  replaceable model FrameMat = Materials.AluminiumFoil constrainedby
    Materials.Properties "Frame material" annotation (choicesAllMatching=true);

  // Geometry
  parameter Length W_cell "Cell width" annotation (Dialog(tab="Geometry"));
  parameter Length H_cell "Cell height" annotation (Dialog(tab="Geometry"));
  parameter Length t_cell "Cell thickness" annotation (Dialog(tab="Geometry"));
  parameter Length t_fw_int "Firewall thickness between adjacent cells" annotation (Dialog(tab="Geometry"));
  parameter Length t_fw_ext "Firewall thickness between cells and frame" annotation (Dialog(tab="Geometry"));
  parameter Length t_gap "Air gap thickness between cells in series" annotation (Dialog(tab="Geometry"));
  parameter Length t_resin "Thickness of resin between cells and frame" annotation (Dialog(tab="Geometry"));
  parameter Length t_frame "Frame thickness" annotation (Dialog(tab="Geometry"));

  // Electrical parameters
  parameter Real eta=0.98 "Cell charging/discharging efficiency";
  parameter ElectricCharge C_nom "Nominal cell capacity";
  parameter Voltage v_nom=3.6 "Nominal voltage";

  // Initialization
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Real SoC_start "State of charge - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart=298.15 "Temperature - start value" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N_cv(min=1)=10 "Number of vertical control volumes in which each cell is discretized";

  Length W_module "Module width";
  Length H_module "Module height";
  Length t_module "Module thickness";
  Volume V_module "Volume of the battery module";

  Components.Electrical.PouchCell cell1(
    redeclare model InPlaneMat = InPlaneCellMat,
    redeclare model CrossPlaneMat = CrossPlaneCellMat,
    W(displayUnit="mm") = W_cell,
    H(displayUnit="mm") = H_cell,
    t(displayUnit="mm") = t_cell,
    eta=eta,
    C_nom=C_nom,
    v_nom=v_nom,
    initOpt=initOpt,
    SoC_start=SoC_start,
    Tstart=Tstart,
    N=N_cv)
    annotation (Placement(transformation(extent={{-176,-34},{-74,34}})));
  Components.Electrical.PouchCell cell2(
    redeclare model InPlaneMat = InPlaneCellMat,
    redeclare model CrossPlaneMat = CrossPlaneCellMat,
    W(displayUnit="mm") = W_cell,
    H(displayUnit="mm") = H_cell,
    t(displayUnit="mm") = t_cell,
    eta=eta,
    C_nom=C_nom,
    v_nom=v_nom,
    initOpt=initOpt,
    SoC_start=SoC_start,
    Tstart=Tstart,
    N=N_cv)
    annotation (Placement(transformation(extent={{-54,-34},{48,34}})));
  Components.Electrical.PouchCell cell3(
    redeclare model InPlaneMat = InPlaneCellMat,
    redeclare model CrossPlaneMat = CrossPlaneCellMat,
    W(displayUnit="mm") = W_cell,
    H(displayUnit="mm") = H_cell,
    t(displayUnit="mm") = t_cell,
    eta=eta,
    C_nom=C_nom,
    v_nom=v_nom,
    initOpt=initOpt,
    SoC_start=SoC_start,
    Tstart=Tstart,
    N=N_cv)
    annotation (Placement(transformation(extent={{64,-34},{166,34}})));

  Components.TwoDimensional.WallConductionHorizontal2D frame_top(
    redeclare model MatX = FrameMat,
    redeclare model MatY = FrameMat,
    x=t_module,
    y=t_frame,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=3)
    annotation (Placement(transformation(extent={{-26,54},{26,106}})));
  Components.TwoDimensional.WallConductionVertical2D firewall12(
    redeclare model MatX = FirewallMat,
    redeclare model MatY = FirewallMat,
    x=t_fw_int,
    y=H_cell,
    z=W_cell,
    Tstart=Tstart,
    initOpt=initOpt,
    N=N_cv)
    annotation (Placement(transformation(extent={{-86,-26},{-34,26}})));
  Components.TwoDimensional.WallConductionVertical2D firewall23(
    redeclare model MatX = FirewallMat,
    redeclare model MatY = FirewallMat,
    x=t_fw_int,
    y=H_cell,
    z=W_cell,
    Tstart=Tstart,
    initOpt=initOpt,
    N=N_cv) annotation (Placement(transformation(extent={{36,-26},{88,26}})));
  Components.TwoDimensional.WallConductionVertical2D firewall1(
    redeclare model MatX = FirewallMat,
    redeclare model MatY = FirewallMat,
    x=t_fw_ext,
    y=H_cell,
    z=W_cell,
    Tstart=Tstart,
    initOpt=initOpt,
    N=N_cv)
    annotation (Placement(transformation(extent={{-206,-26},{-154,26}})));
  Components.TwoDimensional.WallConductionVertical2D firewall3(
    redeclare model MatX = FirewallMat,
    redeclare model MatY = FirewallMat,
    x=t_fw_ext,
    y=H_cell,
    z=W_cell,
    Tstart=Tstart,
    initOpt=initOpt,
    N=N_cv)
    annotation (Placement(transformation(extent={{154,-26},{206,26}})));
  Components.TwoDimensional.WallConductionHorizontal2D resin_top(
    redeclare model MatX = ResinMat,
    redeclare model MatY = ResinMat,
    x=t_module,
    y=t_resin,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=3) annotation (Placement(transformation(extent={{-26,24},{26,76}})));
  Components.TwoDimensional.WallConductionHorizontal2D resin_bottom(
    redeclare model MatX = ResinMat,
    redeclare model MatY = ResinMat,
    x=t_module,
    y=t_resin,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=3) annotation (Placement(transformation(extent={{-26,-76},{26,-24}})));
  Components.TwoDimensional.WallConductionHorizontal2D frame_bottom(
    redeclare model MatX = FrameMat,
    redeclare model MatY = FrameMat,
    x=t_module,
    y=t_frame,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=3) annotation (Placement(transformation(extent={{-26,-106},{26,-54}})));
  Components.TwoDimensional.WallConductionVertical2D frame_right(
    redeclare model MatX = FrameMat,
    redeclare model MatY = FrameMat,
    x=t_frame,
    y=H_module,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=N_cv) annotation (Placement(transformation(extent={{184,-26},{236,26}})));
  Components.TwoDimensional.WallConductionVertical2D frame_left(
    redeclare model MatX = FrameMat,
    redeclare model MatY = FrameMat,
    x=t_frame,
    y=H_module,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=N_cv)
    annotation (Placement(transformation(extent={{-236,-26},{-184,26}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p annotation (Placement(
        transformation(extent={{-248,112},{-232,128}}),iconTransformation(
          extent={{-246,74},{-234,86}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n annotation (Placement(
        transformation(extent={{248,102},{232,118}}), iconTransformation(extent={{234,74},
            {246,86}})));
  Modelica.Electrical.Analog.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{-210,
            110},{-190,130}})));
  Modelica.Electrical.Batteries.Interfaces.CellBus batteryBus
    "Battery bus (average / sum over all cells)" annotation (Placement(
        transformation(extent={{-218,72},{-182,108}}),iconTransformation(extent={{-10,30},
            {10,50}})));

  CustomInterfaces.OneDimensional.HeatPort1D_A North(Nx=3) annotation (
      Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={0,140}), iconTransformation(
        extent={{-40,-40},{40,40}},
        rotation=180,
        origin={0,140})));
  CustomInterfaces.OneDimensional.HeatPort1D_B South(Nx=3) annotation (
      Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={0,-100}), iconTransformation(
        extent={{-40,-40},{40,40}},
        rotation=180,
        origin={0,-100})));
  CustomInterfaces.OneDimensional.HeatPort1D_B West(Nx=N_cv) annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=-90,
        origin={-240,0}), iconTransformation(
        extent={{-40,-26},{40,26}},
        rotation=-90,
        origin={-240,7.10543e-15})));
  CustomInterfaces.OneDimensional.HeatPort1D_A East(Nx=N_cv) annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=-90,
        origin={240,0}), iconTransformation(
        extent={{-40,-24},{40,24}},
        rotation=-90,
        origin={240,0})));

equation
  // Geometry
  W_module = W_cell + 2*t_frame;
  H_module = H_cell + 2*t_resin + 2*t_frame;
  t_module = 3*t_cell + 2*t_fw_int + 2*t_fw_ext + 2*t_frame;
  V_module = W_module*H_module*t_module;

  // -------------------------------- THERMAL ----------------------------------
  connect(resin_top.South.ports[1], cell1.Top);
  connect(resin_top.South.ports[2], cell2.Top);
  connect(resin_top.South.ports[3], cell3.Top);
  connect(resin_bottom.North.ports[1], cell1.Bottom);
  connect(resin_bottom.North.ports[2], cell2.Bottom);
  connect(resin_bottom.North.ports[3], cell3.Bottom);

  // ------------------------------- ELECTRICAL --------------------------------

  connect(firewall1.East, cell1.Left)
    annotation (Line(points={{-172.2,0},{-155.033,0}}, color={191,0,0}));
  connect(cell1.Right, firewall12.West)
    annotation (Line(points={{-93.8333,0},{-67.8,0}}, color={191,0,0}));
  connect(firewall12.East, cell2.Left)
    annotation (Line(points={{-52.2,0},{-33.0333,0}}, color={191,0,0}));
  connect(cell2.Right, firewall23.West)
    annotation (Line(points={{28.1667,0},{54.2,0}}, color={191,0,0}));
  connect(firewall23.East, cell3.Left)
    annotation (Line(points={{69.8,0},{84.9667,0}}, color={191,0,0}));
  connect(cell3.Right, firewall3.West)
    annotation (Line(points={{146.167,0},{172.2,0}}, color={191,0,0}));
  connect(resin_top.North, frame_top.South)
    annotation (Line(points={{0,57.8},{0,72.2}}, color={191,0,0}));
  connect(frame_bottom.North, resin_bottom.South)
    annotation (Line(points={{0,-72.2},{0,-57.8}}, color={191,0,0}));
  connect(frame_left.East,firewall1. West)
    annotation (Line(points={{-202.2,0},{-187.8,0}}, color={191,0,0}));
  connect(firewall3.East, frame_right.West)
    annotation (Line(points={{187.8,0},{202.2,0}}, color={191,0,0}));
  connect(frame_top.East, frame_right.North)
    annotation (Line(points={{23.4,80},{210,80},{210,23.4}}, color={191,0,0}));
  connect(frame_right.South, frame_bottom.East) annotation (Line(points={{210,
          -23.4},{210,-80},{23.4,-80}},
                                 color={191,0,0}));
  connect(frame_bottom.West, frame_left.South) annotation (Line(points={{-23.4,
          -80},{-210,-80},{-210,-23.4}},
                                    color={191,0,0}));
  connect(frame_left.North, frame_top.West) annotation (Line(points={{-210,23.4},
          {-210,80},{-23.4,80}}, color={191,0,0}));
  connect(p, multiSensor.pc) annotation (Line(points={{-240,120},{-210,120}},
                 color={0,0,255}));
  connect(multiSensor.pc, multiSensor.pv) annotation (Line(points={{-210,120},{
          -210,130},{-200,130}}, color={0,0,255}));
  connect(multiSensor.nv, n) annotation (Line(points={{-200,110},{240,110}},
                             color={0,0,255}));
  connect(multiSensor.power, batteryBus.power) annotation (Line(points={{-211,114},
          {-220,114},{-220,90.09},{-199.91,90.09}},      color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(multiSensor.i, batteryBus.i) annotation (Line(points={{-206,109},{-206,
          90.09},{-199.91,90.09}},                 color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(multiSensor.v, batteryBus.v) annotation (Line(points={{-194,109},{-194,
          88},{-199.91,88},{-199.91,90.09}},      color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(multiSensor.nc, cell1.p) annotation (Line(points={{-190,120},{
          -84.7667,120},{-84.7667,10.7667}}, color={0,0,255}));
  connect(cell1.n, cell2.p) annotation (Line(points={{-84.7667,-11.9},{-40,
          -11.9},{-40,34},{37.2333,34},{37.2333,10.7667}}, color={0,0,255}));
  connect(cell2.n, cell3.p) annotation (Line(points={{37.2333,-11.9},{80,-11.9},
          {80,34},{155.233,34},{155.233,10.7667}}, color={0,0,255}));
  connect(cell3.n, n) annotation (Line(points={{155.233,-11.9},{156,-11.9},{156,
          -12},{164,-12},{164,110},{240,110}},
               color={0,0,255}));
  connect(South, frame_bottom.South)
    annotation (Line(points={{0,-100},{0,-87.8}}, color={191,0,0}));
  connect(frame_top.North, North)
    annotation (Line(points={{0,87.8},{0,140}}, color={191,0,0}));
  connect(West, frame_left.West)
    annotation (Line(points={{-240,0},{-217.8,0}}, color={191,0,0}));
  connect(frame_right.East, East)
    annotation (Line(points={{217.8,0},{240,0}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
            -100},{240,140}})),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-240,-100},{240,140}})));
end PouchCellModuleManual2D;
