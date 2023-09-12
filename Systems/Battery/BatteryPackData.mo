within ThermalManagement.Systems.Battery;
record BatteryPackData "Data for battery packs made by stacking multiple lithium-ion cells"

  extends Modelica.Icons.Record;

  parameter Modelica.Units.NonSI.Energy_kWh E_pack "Total energy storage of battery pack (SOC = 100%)";
  parameter Real eta_pack=0.95 "Efficiency of battery pack";

  // Cells' materials [1]

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BatteryPackData;
