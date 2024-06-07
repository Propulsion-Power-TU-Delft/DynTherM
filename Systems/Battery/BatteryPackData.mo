within DynTherM.Systems.Battery;
record BatteryPackData "Data for battery packs made by stacking multiple lithium-ion cells"

  extends Modelica.Icons.Record;

  parameter Modelica.Units.NonSI.Energy_kWh E_pack "Total energy storage of battery pack (SOC = 100%)";
  parameter Real eta_pack=0.95 "Efficiency of battery pack";
  parameter Mass m_tot=34927 "Total mass of the battery packs";
  parameter SpecificHeatCapacity c=1000 "Average specific heat capacity of the battery pack (800-1200)";

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BatteryPackData;
