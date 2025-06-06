within DynTherM.Systems.Aircraft;
model AirbusA320
  "Model of an Airbus A320: cabin + cargo + cockpit (no leakage) + recirculation"

  extends PassengerAircraft(
    X_ECS={0,1},
    R_cabin=2.07,
    R_cockpit=R_cabin,
    L_cabin=26.9,
    L_cockpit=3.5,
    L_cargo=4.95 + 9.8,
    L_EEbay=4,
    V_cabin=139,
    V_cockpit=9,
    V_cargo=15.56 + 20.77,
    V_EEbay=V_cockpit,
    L_window=0.23,
    H_window=0.33,
    L_windshield_front=2*0.52/0.5,
    H_windshield_front=0.5,
    L_windshield_lat=(0.3+0.36)/0.5,
    H_windshield_lat=0.5,
    Nw_side=30,
    H_fl=1.3,
    c_cabin=1000,
    c_cockpit=1000,
    m_cabin=20*180,
    m_cockpit=20*4,
    A_cabin=2*180,
    A_cockpit=2*4,
    t_upper=0.1,
    t_lower=0.1,
    L_pipe_cab=L_cabin/2,
    D_pipe_cab=0.18,
    L_pipe_fd=L_cabin/2,
    D_pipe_fd=0.08,
    V_mixingManifold=0.5,
    eta_is=0.65,
    eta_m=0.95,
    omega_nom=157.08,
    volFlow_nom=0.7,
    Head_nom=400,
    Kv=0.005,
    R_HEPA=2000,
    R_dado=100,
    Roughness=4.5e-05);
  annotation (Icon(coordinateSystem(extent={{-180,-140},{100,100}}),
                   graphics={Bitmap(extent={{-106,-10},{-28,20}}, fileName=
              "modelica://DynTherM/Figures/AirbusLogo.png"),
          Text(
          extent={{-68,6},{-42,-20}},
          lineColor={0,32,91},
          textString="A320",
          textStyle={TextStyle.Bold})}), Diagram(coordinateSystem(extent={{-180,
            -140},{100,100}})),
    Documentation(info="<html>
<p>Reference:</p>
<p>[1] Airbus&nbsp;A320,&nbsp;Aircraft&nbsp;Characteristics&nbsp;Airport&nbsp;and&nbsp;Maintenance&nbsp;Planning.</p>
</html>"));
end AirbusA320;
