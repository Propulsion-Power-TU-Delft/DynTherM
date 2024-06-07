within DynTherM.Components.Battery;
model PouchCell1D
  "Electro-thermal model of a pouch cell featuring 1D discretization"

  // Geometery
  parameter Modelica.Units.SI.Length h = 1 "Cell height" annotation (Dialog(tab="Geometry"));
  parameter Modelica.Units.SI.Area A = 0.025  "Cell Base surface Area" annotation (Dialog(tab="Geometry"));

  // Cell Electrical Parameters
  parameter Modelica.Units.SI.PerUnit eta = 0.98 "Charging/Discharging Efficiency"  annotation (Dialog(tab="Electrical Parameters"));
  parameter Modelica.Units.SI.ElectricCharge capacity = 230400 "Battery Capacity in Ampere-second" annotation (Dialog(tab="Electrical Parameters"));
  parameter Modelica.Units.SI.PerUnit SOC_start = 0.10 "Starting charge state" annotation (Dialog(tab="Electrical Parameters"));

  // Initialization
  parameter Modelica.Units.SI.Temperature Tstart=298.15
    "Temperature start value" annotation (Dialog(tab="Initialization"));
//   parameter DynTherM.Choices.InitOpt initOpt "Initialization option"
//     annotation (Dialog(tab="Initialization"));
  parameter Boolean steadyStateInit = false "=true if steady-state initialization for charging current" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N(min=1)= 10 "Number of vertical sections in which the cell is discretized";


  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-20,38},{-38,56}})));
  Components.HeatTransfer.CellHeatLoss Battery_Model(
        steadyStateInit=steadyStateInit,
        eta = eta,
        capacity = capacity,
        SOC_start = SOC_start)
    annotation (Placement(transformation(extent={{-84,-76},{8,12}})));

  Components.OneDimensional.PouchCellThermal1D
    cell_1D_Discretized(
        A=A,
        h=h,
        Tstart=Tstart,
        initOpt=DynTherM.Choices.InitOpt.fixedState,
    N=N)    annotation (Placement(transformation(extent={{22,14},{80,72}})));

  Modelica.Thermal.HeatTransfer.Celsius.FromKelvin fromKelvin
    annotation (Placement(transformation(extent={{-72,38},{-92,58}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a annotation (
      Placement(transformation(extent={{44,-42},{60,-26}}), iconTransformation(
          extent={{44,-42},{60,-26}})));
  Modelica.Blocks.Interfaces.RealInput Cell_Current annotation (Placement(
        transformation(extent={{-92,-8},{-76,8}}), iconTransformation(
          extent={{-92,-8},{-76,8}})));
equation

  connect(cell_1D_Discretized.AvgT_port, temperatureSensor.port)
    annotation (Line(points={{32.44,43},{-14,43},{-14,47},{-20,47}},
        color={191,0,0}));
  connect(temperatureSensor.T, fromKelvin.Kelvin)
    annotation (Line(points={{-38.9,47},{-40,48},{-70,48}}, color={0,0,127}));
  connect(fromKelvin.Celsius, Battery_Model.Temp) annotation (Line(points={{-93,48},
          {-98,48},{-98,-27.6},{-52.375,-27.6}},       color={0,0,127}));
  connect(Battery_Model.Q_total, cell_1D_Discretized.Heat_gen) annotation (Line(
        points={{-23.9125,-15.72},{-23.9125,-16},{92,-16},{92,42.71},{70.43,
          42.71}},
        color={0,0,127}));
  connect(port_a, cell_1D_Discretized.Convection_Port) annotation (Line(points={{52,-34},
          {51,-34},{51,19.8}},                 color={191,0,0}));
  connect(Cell_Current, Battery_Model.Curr) annotation (Line(points={{-84,0},
          {-76,0},{-76,-76},{-44,-76},{-44,-60.16},{-43.75,-60.16}},
                                         color={0,0,127}));
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
