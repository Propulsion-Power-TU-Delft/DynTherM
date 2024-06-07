within DynTherM.Components.Battery;
model Cell18650
  extends CylindricalCell(
    H_cell=0.065,
    D_cell=0.0184,
    D_pin=0.0025,
    E_cell=11.25,
    M_cell=45e-3,
    Q_vol=84.5e3);
end Cell18650;
