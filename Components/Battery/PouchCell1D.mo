within DynTherM.Components.Battery;
model PouchCell1D
  "Electro-thermal model of a pouch cell featuring 1D discretization"

  replaceable model Mat = Materials.PolestarCell constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);

  // Geometery
  parameter Modelica.Units.SI.Length H = 1 "Cell height" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Area A = 0.025  "Cell Base surface Area" annotation (Dialog(tab="Geometry"));

  // Electrical Parameters of cell
  parameter Real eta = 0.98 "Charging/discharging efficiency";
  parameter ElectricCharge capacity = 216000 "Battery capacity";
  parameter Real SoC_min = 0.00 "Minimum allowable state of charge";
  parameter Real SoC_max = 1.00 "Maximum allowable state of charge";

  // Initialization
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Real SoC_start "Starting state of charge" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart=288.15 "Temperature start value" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N(min=1)= 10 "Number of vertical sections in which the cell is discretized";


  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-32,42},{-52,62}})));
  Components.HeatTransfer.CellHeatLoss Battery_Model(
        eta = eta,
        capacity = capacity,
    SoC_min=SoC_min,
    SoC_max=SoC_max,
    initOpt=initOpt,
    SoC_start=SoC_start)
    annotation (Placement(transformation(extent={{-94,-78},{-2,10}})));

  Components.OneDimensional.PouchCellThermal1D
    cell_1D_Discretized(
    redeclare model Mat = Mat,
        A=A,
    H=H,Tstart=Tstart,
    initOpt=initOpt,
    N=N)    annotation (Placement(transformation(extent={{22,14},{80,72}})));

  Modelica.Blocks.Interfaces.RealInput I annotation (Placement(transformation(
          extent={{-124,-96},{-108,-80}}), iconTransformation(extent={{-124,-96},
            {-108,-80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Top annotation (Placement(
        transformation(extent={{44,76},{56,88}}), iconTransformation(extent={{-10,
            -6},{4,8}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Bottom annotation (
      Placement(transformation(extent={{44,-6},{56,6}}), iconTransformation(
          extent={{-10,-6},{4,8}})));
equation

  connect(I, Battery_Model.I) annotation (Line(points={{-116,-88},{-43.4,-88},{-43.4,
          -73.6}}, color={0,0,127}));
  connect(temperatureSensor.T, Battery_Model.T) annotation (Line(points={{-53,52},
          {-100,52},{-100,-55.12},{-89.975,-55.12}},     color={0,0,127}));
  connect(Battery_Model.Q, cell_1D_Discretized.Q) annotation (Line(points={{-8.0375,
          -13.32},{6,-13.32},{6,43},{24.9,43}}, color={0,0,127}));
  connect(cell_1D_Discretized.Average, temperatureSensor.port) annotation (Line(
        points={{50.13,43.29},{50.13,52},{-32,52}}, color={191,0,0}));
  connect(Top, cell_1D_Discretized.Top) annotation (Line(points={{50,82},{50.13,
          82},{50.13,60.69}}, color={191,0,0}));
  connect(cell_1D_Discretized.Bottom, Bottom)
    annotation (Line(points={{50.13,25.31},{50,24},{50,0}}, color={191,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-180,-100},{140,100}}),
                                                      graphics={Bitmap(
            extent={{-82,-38},{78,38}}, fileName=
              "modelica://Battery_Model/../Battery Cell.png")}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-180,-100},{140,
            100}})),
    experiment(StopTime=1200, __Dymola_Algorithm="Dassl"),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PouchCell1D;
