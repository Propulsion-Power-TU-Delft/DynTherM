within DynTherM.CustomInterfaces;
package Adaptors "Models used to couple two connectors of different type"
  model irradianceToHeatFlow
    "Model used to convert from irradiance port to heat port"

    parameter Modelica.Units.SI.Area A "Heat transfer surface";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
      annotation (Placement(transformation(extent={{-14,46},{14,74}}),
          iconTransformation(extent={{-14,46},{14,74}})));
    CustomInterfaces.IrradiancePort outlet
      annotation (Placement(transformation(extent={{-14,-74},{14,-46}}),
          iconTransformation(extent={{-14,-74},{14,-46}})));
  equation
    inlet.Q_flow + A*(outlet.E_tb + outlet.E_td + outlet.E_tr) = 0;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-34},{0,34}},     color={238,46,47}),
          Line(points={{0,34},{6,24}},      color={238,46,47}),
          Line(points={{0,34},{-6,24}},     color={238,46,47})}),  Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
  end irradianceToHeatFlow;

  model heatFluxToHeatFlow
    "Model used to convert from heat flux port to heat port"

    input Area A "Heat transfer surface";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
      annotation (Placement(transformation(extent={{-14,46},{14,74}}),
          iconTransformation(extent={{-14,46},{14,74}})));
    CustomInterfaces.HeatFluxPort_A outlet
      annotation (Placement(transformation(extent={{-14,-74},{14,-46}}),
          iconTransformation(extent={{-14,-74},{14,-46}})));
  equation
    inlet.Q_flow + A*outlet.phi = 0;
    inlet.T = outlet.T;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-34},{0,34}},     color={238,46,47}),
          Line(points={{0,34},{6,24}},      color={238,46,47}),
          Line(points={{0,34},{-6,24}},     color={238,46,47})}),  Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
  end heatFluxToHeatFlow;

  model irradianceMultiplier
    "Model used to convert from irradiance port to distributed irradiance port"
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    parameter Integer Ny(min=1) "Number of ports in y-direction";

    CustomInterfaces.IrradiancePort single annotation (Placement(transformation(extent={{-14,-74},{14,-46}}),
          iconTransformation(extent={{-14,-74},{14,-46}})));
    CustomInterfaces.DistributedIrradiancePort distributed(Nx=Nx, Ny=Ny) annotation (Placement(transformation(extent={{-40,-10},{40,130}}),
          iconTransformation(extent={{-40,-10},{40,130}})));

  equation
    for i in 1:Nx loop
      for j in 1:Ny loop
        distributed.ports[i,j].E_tb  = single.E_tb;
        distributed.ports[i,j].E_td  = single.E_td;
        distributed.ports[i,j].E_tr  = single.E_tr;
        distributed.ports[i,j].theta = single.theta;
      end for;
    end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-34},{0,34}},     color={238,46,47}),
          Line(points={{0,34},{6,24}},      color={238,46,47}),
          Line(points={{0,34},{-6,24}},     color={238,46,47})}),  Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
  end irradianceMultiplier;

  model heatFlowMultiplier "Model used to convert from heat port to distributed heat port"
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    parameter Integer Ny(min=1) "Number of ports in y-direction";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a single annotation (Placement(transformation(extent={{-14,-74},{14,-46}}),
         iconTransformation(extent={{-14,-74},{14,-46}})));
    CustomInterfaces.DistributedHeatPort_A distributed(Nx=Nx, Ny=Ny) annotation (Placement(transformation(extent={{-40,-10},{40,130}}),
         iconTransformation(extent={{-40,-10},{40,130}})));

  equation
    sum(distributed.ports.Q_flow) + single.Q_flow = 0;

    for i in 1:Nx loop
      for j in 1:Ny loop
        distributed.ports[i,j].T  = single.T;
      end for;
    end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-34},{0,34}},     color={238,46,47}),
          Line(points={{0,34},{6,24}},      color={238,46,47}),
          Line(points={{0,34},{-6,24}},     color={238,46,47})}),  Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p></html>"),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end heatFlowMultiplier;

  model flowScaler
    "Model used to scale up or down the mass flow rate between two fluid ports"

    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
    input Real scaler=1 "Inlet / outlet mass flow rate" annotation(Dialog(enable = true));

    CustomInterfaces.FluidPort_A inlet(redeclare package Medium = Medium)
      annotation (Placement(transformation(extent={{-10,-70},{10,-50}})));
    CustomInterfaces.FluidPort_B outlet(redeclare package Medium = Medium)
      annotation (Placement(transformation(extent={{-10,50},{10,70}})));

  equation
    inlet.m_flow + scaler*outlet.m_flow = 0 "Mass balance";
    inlet.P = outlet.P "Momentum balance";

    // Energy balance
    inlet.h_outflow = inStream(outlet.h_outflow);
    outlet.h_outflow = inStream(inlet.h_outflow);

    // Independent composition mass balances
    inlet.Xi_outflow = inStream(outlet.Xi_outflow);
    outlet.Xi_outflow = inStream(inlet.Xi_outflow);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-34},{0,34}},     color={0,0,0}),
          Line(points={{0,34},{6,24}},      color={0,0,0}),
          Line(points={{0,34},{-6,24}},     color={0,0,0}),
          Line(points={{-10,-10},{-30,10}}, color={0,0,0}),
          Line(points={{-10,10},{-30,-10}}, color={0,0,0})}),      Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
  end flowScaler;
end Adaptors;
