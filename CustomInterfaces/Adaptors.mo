within DynTherM.CustomInterfaces;
package Adaptors "Models used to couple two connectors of different type"
  model irradianceToHeatFlow
    "Model used to convert from irradiance port to heat port"

    parameter Area A "Heat transfer surface";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
      annotation (Placement(transformation(extent={{-14,46},{14,74}}),
          iconTransformation(extent={{-14,46},{14,74}})));
    ZeroDimensional.IrradiancePort outlet annotation (Placement(transformation(
            extent={{-14,-74},{14,-46}}), iconTransformation(extent={{-14,-74},
              {14,-46}})));

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
    ZeroDimensional.HeatFluxPort_A outlet annotation (Placement(transformation(
            extent={{-14,-74},{14,-46}}), iconTransformation(extent={{-14,-74},
              {14,-46}})));

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

  model irradianceMultiplier1D
    "Convert from 0D irradiance port to 1D irradiance port"
    parameter Integer Nx(min=1) "Number of ports in x-direction";

    ZeroDimensional.IrradiancePort single annotation (Placement(transformation(
            extent={{-14,-74},{14,-46}}), iconTransformation(extent={{-14,-74},
              {14,-46}})));
    OneDimensional.IrradiancePort1D distributed(Nx=Nx) annotation (
        Placement(transformation(extent={{-40,-10},{40,130}}), iconTransformation(
            extent={{-40,-10},{40,130}})));

  equation
    for i in 1:Nx loop
      distributed.ports[i].E_tb  = single.E_tb;
      distributed.ports[i].E_td  = single.E_td;
      distributed.ports[i].E_tr  = single.E_tr;
      distributed.ports[i].theta = single.theta;
    end for;


    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-34},{0,34}},     color={238,46,47}),
          Line(points={{0,34},{6,24}},      color={238,46,47}),
          Line(points={{0,34},{-6,24}},     color={238,46,47})}),  Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
  end irradianceMultiplier1D;

  model irradianceMultiplier2D
    "Convert from 0D irradiance port to 2D irradiance port"
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    parameter Integer Ny(min=1) "Number of ports in y-direction";

    ZeroDimensional.IrradiancePort single annotation (Placement(transformation(
            extent={{-14,-74},{14,-46}}), iconTransformation(extent={{-14,-74},
              {14,-46}})));
    TwoDimensional.IrradiancePort2D distributed(Nx=Nx, Ny=Ny) annotation (
        Placement(transformation(extent={{-40,-10},{40,130}}), iconTransformation(
            extent={{-40,-10},{40,130}})));

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
  end irradianceMultiplier2D;

  model irradianceMultiplier3D
    "Convert from 0D irradiance port to 3D irradiance port"
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    parameter Integer Ny(min=1) "Number of ports in y-direction";
    parameter Integer Nz(min=1) "Number of ports in z-direction";

    ZeroDimensional.IrradiancePort single annotation (Placement(transformation(
            extent={{-14,-74},{14,-46}}), iconTransformation(extent={{-14,-74},
              {14,-46}})));
    ThreeDimensional.IrradiancePort3D distributed(Nx=Nx, Ny=Ny, Nz=Nz) annotation (
        Placement(transformation(extent={{-40,-10},{40,130}}), iconTransformation(
            extent={{-40,-10},{40,130}})));

  equation
    for i in 1:Nx loop
      for j in 1:Ny loop
        for k in 1:Nz loop
            distributed.ports[i,j,k].E_tb  = single.E_tb;
            distributed.ports[i,j,k].E_td  = single.E_td;
            distributed.ports[i,j,k].E_tr  = single.E_tr;
            distributed.ports[i,j,k].theta = single.theta;
          end for;
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
  end irradianceMultiplier3D;

  model heatFlowMultiplier1D
    "Model used to convert from 0D heat port to 1D heat port"
    parameter Integer Nx(min=1) "Number of ports in x-direction";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a single annotation (Placement(transformation(extent={{-14,-74},{14,-46}}),
         iconTransformation(extent={{-14,-74},{14,-46}})));
    OneDimensional.HeatPort1D_A distributed(Nx=Nx) annotation (Placement(
          transformation(extent={{-40,-10},{40,130}}), iconTransformation(extent={
              {-40,-10},{40,130}})));

  equation
    sum(distributed.ports.Q_flow) + single.Q_flow = 0;

    for i in 1:Nx loop
      distributed.ports[i].T = single.T;
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
  end heatFlowMultiplier1D;

  model heatFlowMultiplier2D
    "Model used to convert from 0D heat port to 2D heat port"
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    parameter Integer Ny(min=1) "Number of ports in y-direction";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a single annotation (Placement(transformation(extent={{-14,-74},{14,-46}}),
         iconTransformation(extent={{-14,-74},{14,-46}})));
    TwoDimensional.HeatPort2D_A distributed(Nx=Nx, Ny=Ny) annotation (Placement(
          transformation(extent={{-40,-10},{40,130}}), iconTransformation(extent={
              {-40,-10},{40,130}})));

  equation
    sum(distributed.ports.Q_flow) + single.Q_flow = 0;

    for i in 1:Nx loop
      for j in 1:Ny loop
        distributed.ports[i,j].T = single.T;
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
  end heatFlowMultiplier2D;

  model heatFlowMultiplier3D
    "Model used to convert from 0D heat port to 3D heat port"
    parameter Integer Nx(min=1) "Number of ports in x-direction";
    parameter Integer Ny(min=1) "Number of ports in y-direction";
    parameter Integer Nz(min=1) "Number of ports in z-direction";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a single annotation (Placement(transformation(extent={{-14,-74},{14,-46}}),
         iconTransformation(extent={{-14,-74},{14,-46}})));
    ThreeDimensional.HeatPort3D_A distributed(Nx=Nx, Ny=Ny, Nz=Nz) annotation (Placement(
          transformation(extent={{-40,-10},{40,130}}), iconTransformation(extent={
              {-40,-10},{40,130}})));

  equation
    sum(distributed.ports.Q_flow) + single.Q_flow = 0;

    for i in 1:Nx loop
      for j in 1:Ny loop
        for k in 1:Nz loop
          distributed.ports[i,j,k].T = single.T;
        end for;
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
  end heatFlowMultiplier3D;

  model heatFlowConverter
    "Model used to convert between distributed heat ports featuring different size"

    model Multiplier = heatFlowMultiplier2D;

    parameter Integer Nx_s1(min=1) "Number of ports in x-direction - side 1";
    parameter Integer Ny_s1(min=1) "Number of ports in y-direction - side 1";
    parameter Integer s1_s2_x_ratio(min=1) "Ratio between no. of ports in x direction s1/s2";
    parameter Integer s1_s2_y_ratio(min=1) "Ratio between no. of ports in y direction s1/s2";
    final parameter Integer Nx_s2 = integer(Nx_s1/s1_s2_x_ratio) "Number of ports in x-direction - side 2";
    final parameter Integer Ny_s2 = integer(Ny_s1/s1_s2_y_ratio) "Number of ports in y-direction - side 2";

    Multiplier multiplier[Nx_s2,Ny_s2](
      each Nx=s1_s2_x_ratio,
      each Ny=s1_s2_y_ratio);

    TwoDimensional.HeatPort2D_A side1(Nx=Nx_s1, Ny=Ny_s1) annotation (Placement(
          transformation(extent={{-40,-130},{40,10}}), iconTransformation(extent={
              {-40,-130},{40,10}})));
    TwoDimensional.HeatPort2D_A side2(Nx=Nx_s2, Ny=Ny_s2) annotation (Placement(
          transformation(extent={{-40,-10},{40,130}}), iconTransformation(extent={
              {-40,-10},{40,130}})));

  equation
    for i in 1:Nx_s2 loop
      for j in 1:Ny_s2 loop
        connect(multiplier[i,j].single, side2.ports[i,j]);
        for k in 1:s1_s2_x_ratio loop
          for z in 1:s1_s2_y_ratio loop
            connect(multiplier[i,j].distributed.ports[k,z],
              side1.ports[(i - 1)*s1_s2_x_ratio + k,(j - 1)*s1_s2_y_ratio + z]);
          end for;
        end for;
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
  end heatFlowConverter;

  model heatFlowScaler
    "Model used to scale up or down the heat flow rate between two heat ports"

    input Real scaler=1 "Inlet / outlet heat flow rate" annotation(Dialog(enable = true));

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet annotation (Placement(transformation(extent={{-10,-70},{10,-50}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet annotation (Placement(transformation(extent={{-10,50},{10,70}})));

  equation
    inlet.Q_flow + scaler*outlet.Q_flow = 0;
    inlet.T = outlet.T;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-34},{0,34}},     color={238,46,47}),
          Line(points={{0,34},{6,24}},      color={238,46,47}),
          Line(points={{0,34},{-6,24}},     color={238,46,47}),
          Line(points={{-10,-10},{-30,10}}, color={238,46,47}),
          Line(points={{-10,10},{-30,-10}}, color={238,46,47})}),  Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
  end heatFlowScaler;

  model flowScaler
    "Model used to scale up or down the mass flow rate between two fluid ports"

    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
    input Real scaler=1 "Inlet / outlet mass flow rate" annotation(Dialog(enable = true));

    ZeroDimensional.FluidPort_A inlet(redeclare package Medium = Medium)
      annotation (Placement(transformation(extent={{-10,-70},{10,-50}})));
    ZeroDimensional.FluidPort_B outlet(redeclare package Medium = Medium)
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

  model heatFlowInverter1D
    "Swap the ports' positions along x-direction"

    parameter Integer Nx(min=1) "Number of volumes in x-direction";

    OneDimensional.HeatPort1D_A input1D(Nx=Nx) annotation (Placement(
          transformation(extent={{-52,16},{54,122}}), iconTransformation(extent={{
              -52,16},{54,122}})));
    OneDimensional.HeatPort1D_A output1D(Nx=Nx) annotation (Placement(
          transformation(extent={{-52,-122},{54,-16}}), iconTransformation(extent
            ={{-52,-122},{54,-16}})));

  equation
    // Connect heat ports with inverted positions
    for i in 1:Nx loop
      connect(input1D.ports[i], output1D.ports[Nx + 1 - i]);
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
  end heatFlowInverter1D;

  model heatFlowInverter2D "Swap the ports' positions along x-direction"

    parameter Integer Nx(min=1) "Number of volumes in x-direction";
    parameter Integer Ny(min=1) "Number of volumes in y-direction";

    TwoDimensional.HeatPort2D_A input2D(Nx=Nx, Ny=Ny) annotation (Placement(
          transformation(extent={{-52,16},{54,122}}), iconTransformation(extent={{
              -52,16},{54,122}})));
    TwoDimensional.HeatPort2D_A output2D(Nx=Nx, Ny=Ny) annotation (Placement(
          transformation(extent={{-52,-122},{54,-16}}), iconTransformation(extent
            ={{-52,-122},{54,-16}})));

  equation
    // Connect heat ports with inverted positions
    for i in 1:Nx loop
      for j in 1:Ny loop
        connect(input2D.ports[i, j], output2D.ports[Nx + 1 - i, j]);
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
  end heatFlowInverter2D;
end Adaptors;
