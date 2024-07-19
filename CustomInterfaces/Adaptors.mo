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

  model heatFlowConverter
    "Model used to convert between distributed heat ports featuring different size"
    parameter Integer Nx_s1(min=1) "Number of ports in x-direction - side 1";
    parameter Integer Ny_s1(min=1) "Number of ports in y-direction - side 1";
    parameter Integer s1_s2_x_ratio(min=1) "Ratio between no. of ports in x direction s1/s2";
    parameter Integer s1_s2_y_ratio(min=1) "Ratio between no. of ports in y direction s1/s2";
    final parameter Integer Nx_s2 = integer(Nx_s1/s1_s2_x_ratio) "Number of ports in x-direction - side 2";
    final parameter Integer Ny_s2 = integer(Ny_s1/s1_s2_y_ratio) "Number of ports in y-direction - side 2";

    CustomInterfaces.DistributedHeatPort_A side1(Nx=Nx_s1, Ny=Ny_s1) annotation (Placement(transformation(extent={{-40,
              -130},{40,10}}),
         iconTransformation(extent={{-40,-130},{40,10}})));
    CustomInterfaces.DistributedHeatPort_A side2(Nx=Nx_s2, Ny=Ny_s2) annotation (Placement(transformation(extent={{-40,-10},{40,130}}),
         iconTransformation(extent={{-40,-10},{40,130}})));

  equation
    assert((s1_s2_x_ratio > 1) and (s1_s2_y_ratio > 1), "The convertion only works in one dimension - not in x and y simultaneously");

    if s1_s2_x_ratio > 1 then
      for j in 1:Ny_s2 loop
        for i in 1:Nx_s2 loop
          for k in 1:s1_s2_x_ratio loop
            connect(side1.ports[k,j], side2.ports[i,j]);
          end for;
        end for;
      end for;
    end if;

    if s1_s2_y_ratio > 1 then
      for i in 1:Nx_s2 loop
        for j in 1:Ny_s2 loop
          for k in 1:s1_s2_y_ratio loop
            connect(side1.ports[i,k], side2.ports[i,j]);
          end for;
        end for;
      end for;
    end if;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-34},{0,34}},     color={238,46,47}),
          Line(points={{0,34},{6,24}},      color={238,46,47}),
          Line(points={{0,34},{-6,24}},     color={238,46,47})}),  Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p></html>"),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end heatFlowConverter;

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

  model InvertDistributedHeatflows "This model flips the position temperature and heat flows across the center row i. 
  For an array of size Nx*Ny, the port.[i,j] becomes port.[Nx+1-i,j]"

    parameter Integer Nx(min=1) "Number of volumes in x-direction";
    parameter Integer Ny(min=1) "Number of volumes in y-direction";

    // Define distributed heat ports
    CustomInterfaces.DistributedHeatPort_A distributedHeatPort_in(Nx=Nx,  Ny=Ny) annotation (
      Placement(transformation(extent={{-52,16},{54,122}}), iconTransformation(
          extent={{-52,16},{54,122}})));

    CustomInterfaces.DistributedHeatPort_A distributedHeatPort_out(Nx=Nx, Ny=Ny) annotation (
      Placement(transformation(extent={{-52,-122},{54,-16}}), iconTransformation(
          extent={{-52,-122},{54,-16}})));

  equation
    // Connect heat ports with inverted positions
    for i in 1:Nx loop
      for j in 1:Ny loop
        connect(distributedHeatPort_in.ports[i,j], distributedHeatPort_out.ports[Nx+1-i, j]);
      end for;
    end for;

    annotation (
      Line(points={{-2,26},{-2,-27},{-1,-27}}, color={191,0,0}),
      Icon(
        graphics={
          Rectangle(extent={{-26,58},{-18,-58}}, fillColor={0,0,0}, fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Polygon(points={{-26,36},{-26,58},{-50,36},{-26,36}}, pattern=LinePattern.None, fillColor={0,0,0}, fillPattern=FillPattern.Solid),
          Rectangle(extent={{20,58},{28,-58}}, fillColor={0,0,0}, fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Polygon(points={{-18,36},{-18,58},{6,36},{-18,36}}, pattern=LinePattern.None, fillColor={0,0,0}, fillPattern=FillPattern.Solid),
          Polygon(points={{28,-36},{28,-58},{52,-36},{28,-36}}, pattern=LinePattern.None, fillColor={0,0,0}, fillPattern=FillPattern.Solid),
          Polygon(points={{20,-36},{20,-58},{-4,-36},{20,-36}}, pattern=LinePattern.None, fillColor={0,0,0}, fillPattern=FillPattern.Solid)}),
      Icon(coordinateSystem(preserveAspectRatio=false)),
      Diagram(coordinateSystem(preserveAspectRatio=false)));
  end InvertDistributedHeatflows;
end Adaptors;
