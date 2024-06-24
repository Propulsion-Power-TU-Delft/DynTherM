within DynTherM.Materials;
model PolestarCellCrossPlane
  "Material used for battery cells of Polestar - cross-plane thermal properties"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2740,
    lambda=0.8,
    cm=1204);
  annotation (Documentation(info="<html>
</html>"));
end PolestarCellCrossPlane;
