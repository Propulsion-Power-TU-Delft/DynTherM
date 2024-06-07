within DynTherM.Systems.Battery;
model CylindricalPack
  "1D thermal model of a battery pack made of cylindrical cells"

  parameter Integer N_series=3 "Number of cells in series";
  parameter Integer N_parallel=2 "Number of cells in parallel";
  parameter Real SOC "State-of-charge [%]";

  // Initialization
  parameter Modelica.Units.SI.Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt "Initialization option"
    annotation (Dialog(tab="Initialization"));

  replaceable model CylindricalCell =
      DynTherM.Components.Battery.Cell18650 constrainedby
    DynTherM.Components.Battery.CylindricalCell;

  CylindricalCell Cell[N_series,N_parallel](
    each SOC=SOC,
    each Tstart=Tstart,
    each initOpt=initOpt);

equation

  // N-S connection
  for j in 1:N_parallel loop
    for i in 1:(N_series-1) loop
      connect(Cell[i,j].N, Cell[i+1,j].S);
    end for;
  end for;

  // E-W connection
  for i in 1:N_series loop
    for j in 1:(N_parallel-1) loop
      connect(Cell[i,j].E, Cell[i,j+1].W);
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-80,-40},{-40,-80}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{40,-40},{80,-80}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,-40},{0,-80}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-80,0},{-40,-40}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-80,80},{-40,40}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,0},{0,-40}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{40,0},{80,-40}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,80},{0,40}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{40,80},{80,40}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-74,-6},{-46,-34}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-34,-6},{-6,-34}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-34,-46},{-6,-74}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-74,-46},{-46,-74}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{46,-6},{74,-34}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{46,-46},{74,-74}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-74,74},{-46,46}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-34,74},{-6,46}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{46,74},{74,46}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{-62,12},{-58,8}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{-62,22},{-58,18}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{-62,32},{-58,28}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{-22,12},{-18,8}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{-22,22},{-18,18}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{-22,32},{-18,28}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{58,12},{62,8}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{58,22},{62,18}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{58,32},{62,28}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{18,-18},{22,-22}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{8,-18},{12,-22}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{28,-18},{32,-22}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{18,-58},{22,-62}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{8,-58},{12,-62}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
                                   Rectangle(
          extent={{28,-58},{32,-62}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CylindricalPack;
