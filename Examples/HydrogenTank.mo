within DynTherM.Examples;
package HydrogenTank

  model FlyingV
    Systems.HydrogenTank.TankControlled tankControlled(
      redeclare model HTC_ext =
          DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.AircraftFlying
          (
          L=10,
          R_ext=2,
          L_nose=42.5),
      L_tank=10,
      R_int=1.997,
      AR=1,
      t_tank(displayUnit="mm") = 0.052115,
      t_insulation=0.16,
      Ff=0.849,
      P_vent=175000,
      f_vent_max=10,
      T_start_tank(displayUnit="K") = 230,
      T_start_insulation(displayUnit="K") = 230,
      P_start=175000,
      E_tb={259.64,798.63,869.79,431.44,0,0,0,0},
      E_td={206.74,118.04,81.35,117.63,206.14,295.66,333.16,296.13},
      E_tr={165.84,48.57,0,48.57,165.84,283.11,331.69,283.11},
      theta={1.300479732246,0.60697315396607,0.46335000981945,1.1110416952346,
          1.8411129213438,2.5346194996237,2.6782426437703,2.0305509583552})
      annotation (Placement(transformation(extent={{-40,-30},{40,32}})));
    inner Components.Environment environment(
      altitude_di(displayUnit="km") = 13000,
      Mach_inf_di=0.85,
      psi=1.5707963267949,
      use_ext_sw=true,
      initOpt=DynTherM.Choices.InitOpt.steadyState)
      annotation (Placement(transformation(extent={{32,50},{80,98}})));
    Modelica.Blocks.Sources.Constant const(k=0)
      annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
    BoundaryConditions.flow_source_ext flow_source_ext(
      P_nom=175000,
      T_nom(displayUnit="K") = 20.268,
      massFlow_nom=-0.39399/2)
      annotation (Placement(transformation(extent={{-90,-24},{-70,-4}})));
    BoundaryConditions.flow_source_ext flow_source_ext1(
      P_nom=175000,
      T_nom(displayUnit="K") = 23,
      massFlow_nom=0)
      annotation (Placement(transformation(extent={{90,10},{70,30}})));
  equation
    connect(const.y, tankControlled.Q_RTBC) annotation (Line(points={{-69,50},{
            -56,50},{-56,26.3636},{-42.2857,26.3636}},
                                                   color={0,0,127}));
    connect(flow_source_ext.outlet, tankControlled.extFluidPort_B) annotation (
        Line(points={{-70,-14},{-39,-14},{-39,-13.0909},{-40,-13.0909}}, color={0,
            0,0}));
    connect(tankControlled.extFluidPort_B1, flow_source_ext1.outlet)
      annotation (Line(points={{40,20.7273},{40,20},{70,20}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end FlyingV;
end HydrogenTank;
